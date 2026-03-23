# /// script
# requires-python = ">=3.10"
# dependencies = ["bleak>=0.21", "aiohttp>=3.9"]
# ///
"""Mac-side BLE relay — HTTP server that forwards commands to Jetson via BLE.

Run with:  uv run scripts/ble_relay.py

Endpoints:
  POST /cmd_vel   {"lx": float, "az": float}  → BLE GATT write (8 bytes)
  POST /e_stop    {"stop": bool}               → BLE GATT write (1 byte)
  GET  /status    → {"connected": bool, "latency_ms": float, "writes_per_sec": float}
"""

from __future__ import annotations

import asyncio
import json
import logging
import struct
import time

from aiohttp import web
from bleak import BleakClient, BleakScanner

logger = logging.getLogger(__name__)

# Must match ble_bridge.py on Jetson
SERVICE_UUID = "a1b2c3d4-e5f6-7890-abcd-ef1234567890"
CMD_VEL_UUID = "a1b2c3d4-e5f6-7890-abcd-ef1234567891"
E_STOP_UUID = "a1b2c3d4-e5f6-7890-abcd-ef1234567892"
HEARTBEAT_UUID = "a1b2c3d4-e5f6-7890-abcd-ef1234567893"

HTTP_PORT = 4200
SCAN_TIMEOUT_S = 10.0
RECONNECT_DELAY_S = 2.0


class BLERelay:
    def __init__(self) -> None:
        self._client: BleakClient | None = None
        self._connected = False
        self._lock = asyncio.Lock()
        self._write_count = 0
        self._write_count_start = time.monotonic()
        self._last_latency_ms = 0.0
        self._reconnect_task: asyncio.Task | None = None

    @property
    def connected(self) -> bool:
        return self._connected and self._client is not None and self._client.is_connected

    @property
    def writes_per_sec(self) -> float:
        elapsed = time.monotonic() - self._write_count_start
        if elapsed < 1.0:
            return 0.0
        rate = self._write_count / elapsed
        # Reset counter periodically to get recent rate
        if elapsed > 10.0:
            self._write_count = 0
            self._write_count_start = time.monotonic()
        return rate

    async def connect(self) -> None:
        """Scan for and connect to the Jetson BLE GATT server."""
        while True:
            try:
                logger.info(f"Scanning for BLE device with service {SERVICE_UUID}...")
                device = await BleakScanner.find_device_by_filter(
                    lambda d, ad: SERVICE_UUID.lower()
                    in [s.lower() for s in (ad.service_uuids or [])],
                    timeout=SCAN_TIMEOUT_S,
                )
                if device is None:
                    logger.warning(
                        f"No device found advertising {SERVICE_UUID}, retrying in {RECONNECT_DELAY_S}s..."
                    )
                    await asyncio.sleep(RECONNECT_DELAY_S)
                    continue

                logger.info(f"Found device: {device.name} ({device.address})")
                client = BleakClient(
                    device,
                    disconnected_callback=self._on_disconnect,
                )
                await client.connect()
                self._client = client
                self._connected = True
                logger.info(f"Connected to {device.name}")

                # Subscribe to heartbeat notifications
                await client.start_notify(HEARTBEAT_UUID, self._on_heartbeat)
                return

            except Exception as e:
                logger.error(f"Connection failed: {e}, retrying in {RECONNECT_DELAY_S}s...")
                await asyncio.sleep(RECONNECT_DELAY_S)

    def _on_disconnect(self, client: BleakClient) -> None:
        logger.warning("BLE disconnected")
        self._connected = False
        # Schedule reconnect
        if self._reconnect_task is None or self._reconnect_task.done():
            self._reconnect_task = asyncio.get_event_loop().create_task(self._reconnect())

    async def _reconnect(self) -> None:
        await asyncio.sleep(RECONNECT_DELAY_S)
        logger.info("Attempting BLE reconnect...")
        await self.connect()

    def _on_heartbeat(self, _char: int, data: bytearray) -> None:
        logger.debug(f"Heartbeat: seq={data[0]}")

    async def write_cmd_vel(self, linear_x: float, angular_z: float) -> bool:
        """Write cmd_vel to BLE characteristic. Returns True on success."""
        if not self.connected or self._client is None:
            return False
        payload = struct.pack("<ff", linear_x, angular_z)
        t0 = time.monotonic()
        try:
            async with self._lock:
                await self._client.write_gatt_char(CMD_VEL_UUID, payload, response=False)
            self._last_latency_ms = (time.monotonic() - t0) * 1000
            self._write_count += 1
            return True
        except Exception as e:
            logger.error(f"BLE write cmd_vel failed: {e}")
            return False

    async def write_e_stop(self, stop: bool) -> bool:
        """Write e_stop to BLE characteristic. Uses write-with-response for safety."""
        if not self.connected or self._client is None:
            return False
        payload = bytes([0x01 if stop else 0x00])
        try:
            async with self._lock:
                await self._client.write_gatt_char(E_STOP_UUID, payload, response=True)
            return True
        except Exception as e:
            logger.error(f"BLE write e_stop failed: {e}")
            return False


# ── HTTP Server ──────────────────────────────────────────────────────────────


def _cors_headers() -> dict[str, str]:
    return {
        "Access-Control-Allow-Origin": "*",
        "Access-Control-Allow-Methods": "GET, POST, OPTIONS",
        "Access-Control-Allow-Headers": "Content-Type",
    }


def _make_app(relay: BLERelay) -> web.Application:
    async def handle_cmd_vel(request: web.Request) -> web.Response:
        try:
            data = await request.json()
            lx = float(data.get("lx", 0.0))
            az = float(data.get("az", 0.0))
        except (json.JSONDecodeError, ValueError, TypeError):
            return web.json_response(
                {"error": "expected JSON {lx, az}"}, status=400, headers=_cors_headers()
            )
        ok = await relay.write_cmd_vel(lx, az)
        return web.json_response({"ok": ok}, headers=_cors_headers())

    async def handle_e_stop(request: web.Request) -> web.Response:
        try:
            data = await request.json()
            stop = bool(data.get("stop", True))
        except (json.JSONDecodeError, ValueError, TypeError):
            return web.json_response(
                {"error": "expected JSON {stop}"}, status=400, headers=_cors_headers()
            )
        ok = await relay.write_e_stop(stop)
        return web.json_response({"ok": ok}, headers=_cors_headers())

    async def handle_status(_request: web.Request) -> web.Response:
        return web.json_response(
            {
                "connected": relay.connected,
                "latency_ms": round(relay._last_latency_ms, 2),
                "writes_per_sec": round(relay.writes_per_sec, 1),
            },
            headers=_cors_headers(),
        )

    async def handle_options(_request: web.Request) -> web.Response:
        return web.Response(status=204, headers=_cors_headers())

    app = web.Application()
    app.router.add_post("/cmd_vel", handle_cmd_vel)
    app.router.add_post("/e_stop", handle_e_stop)
    app.router.add_get("/status", handle_status)
    # CORS preflight
    app.router.add_route("OPTIONS", "/cmd_vel", handle_options)
    app.router.add_route("OPTIONS", "/e_stop", handle_options)
    return app


async def _main() -> None:
    relay = BLERelay()
    logger.info("Starting BLE relay — scanning for Jetson...")
    await relay.connect()

    app = _make_app(relay)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "127.0.0.1", HTTP_PORT)
    await site.start()
    logger.info(f"HTTP server listening on http://127.0.0.1:{HTTP_PORT}")
    logger.info("Endpoints: POST /cmd_vel, POST /e_stop, GET /status")

    # Run forever
    try:
        while True:
            await asyncio.sleep(3600)
    except asyncio.CancelledError:
        pass
    finally:
        await runner.cleanup()


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    )
    try:
        asyncio.run(_main())
    except KeyboardInterrupt:
        logger.info("BLE relay shutting down")


if __name__ == "__main__":
    main()
