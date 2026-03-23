# /// script
# requires-python = ">=3.10"
# dependencies = ["bless>=0.2.6", "aiohttp>=3.9"]
# ///
"""Mac-side BLE server — advertises a GATT service and relays commands to Jetson.

The Mac runs as BLE peripheral (bless works natively on macOS via CoreBluetooth).
The Jetson connects as BLE central (bleak) and subscribes to notifications.

Foxglove panel POSTs gamepad data to localhost:4200 → this process updates the
GATT characteristic → BLE notification → Jetson receives and publishes to ROS2.

Run with:  uv run scripts/ble_relay.py

Endpoints:
  POST /cmd_vel   {"lx": float, "az": float}  → BLE notification (8 bytes)
  POST /e_stop    {"stop": bool}               → BLE notification (1 byte)
  GET  /status    → {"connected": bool, "latency_ms": float, "writes_per_sec": float}
"""

from __future__ import annotations

import asyncio
import json
import logging
import struct
import time

from aiohttp import web
from bless import (
    BlessGATTCharacteristic,
    BlessServer,
    GATTAttributePermissions,
    GATTCharacteristicProperties,
)

logger = logging.getLogger(__name__)

# Must match ble_bridge.py on Jetson
SERVICE_UUID = "a1b2c3d4-e5f6-7890-abcd-ef1234567890"
CMD_VEL_UUID = "a1b2c3d4-e5f6-7890-abcd-ef1234567891"
E_STOP_UUID = "a1b2c3d4-e5f6-7890-abcd-ef1234567892"
HEARTBEAT_UUID = "a1b2c3d4-e5f6-7890-abcd-ef1234567893"

DEVICE_NAME = "CouchVision-BLE"
HTTP_PORT = 4200


class BLEPeripheral:
    def __init__(self) -> None:
        self._server: BlessServer | None = None
        self._connected = False
        self._write_count = 0
        self._write_count_start = time.monotonic()
        self._last_latency_ms = 0.0
        # Cached after start() to avoid per-notify lookups
        self._cmd_vel_char: BlessGATTCharacteristic | None = None
        self._e_stop_char: BlessGATTCharacteristic | None = None
        self._heartbeat_char: BlessGATTCharacteristic | None = None

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def latency_ms(self) -> float:
        return self._last_latency_ms

    @property
    def writes_per_sec(self) -> float:
        elapsed = time.monotonic() - self._write_count_start
        if elapsed < 1.0:
            return 0.0
        rate = self._write_count / elapsed
        if elapsed > 10.0:
            self._write_count = 0
            self._write_count_start = time.monotonic()
        return rate

    def _on_read(self, characteristic: BlessGATTCharacteristic, **kwargs) -> bytearray:
        return characteristic.value

    def _on_write(self, characteristic: BlessGATTCharacteristic, value: bytearray, **kwargs) -> None:
        logger.debug(f"Write from central: uuid={characteristic.uuid} len={len(value)}")

    async def start(self) -> None:
        """Start the BLE GATT server and advertise."""
        server = BlessServer(name=DEVICE_NAME)
        server.read_request_func = self._on_read
        server.write_request_func = self._on_write

        await server.add_new_service(SERVICE_UUID)

        char_props = (
            GATTCharacteristicProperties.read
            | GATTCharacteristicProperties.notify
        )
        char_perms = GATTAttributePermissions.readable

        for uuid in (CMD_VEL_UUID, E_STOP_UUID, HEARTBEAT_UUID):
            await server.add_new_characteristic(
                SERVICE_UUID, uuid, char_props, None, char_perms,
            )

        await server.start()
        self._server = server
        self._cmd_vel_char = server.get_characteristic(CMD_VEL_UUID)
        self._e_stop_char = server.get_characteristic(E_STOP_UUID)
        self._heartbeat_char = server.get_characteristic(HEARTBEAT_UUID)
        self._connected = True
        logger.info(f"BLE peripheral '{DEVICE_NAME}' advertising service {SERVICE_UUID}")

    def notify_cmd_vel(self, linear_x: float, angular_z: float) -> bool:
        """Update CMD_VEL characteristic and send BLE notification."""
        if self._cmd_vel_char is None:
            return False
        t0 = time.monotonic()
        self._cmd_vel_char.value = bytearray(struct.pack("<ff", linear_x, angular_z))
        self._server.update_value(SERVICE_UUID, CMD_VEL_UUID)
        self._last_latency_ms = (time.monotonic() - t0) * 1000
        self._write_count += 1
        return True

    def notify_e_stop(self, stop: bool) -> bool:
        """Update E_STOP characteristic and send BLE notification."""
        if self._e_stop_char is None:
            return False
        self._e_stop_char.value = bytearray([0x01 if stop else 0x00])
        self._server.update_value(SERVICE_UUID, E_STOP_UUID)
        return True

    async def heartbeat_loop(self) -> None:
        """Send heartbeat notifications every 500ms."""
        seq = 0
        while True:
            await asyncio.sleep(0.5)
            if self._heartbeat_char is None:
                continue
            seq = (seq + 1) % 256
            self._heartbeat_char.value = bytearray([seq])
            self._server.update_value(SERVICE_UUID, HEARTBEAT_UUID)


# -- HTTP Server ---------------------------------------------------------------


_CORS_HEADERS = {
    "Access-Control-Allow-Origin": "*",
    "Access-Control-Allow-Methods": "GET, POST, OPTIONS",
    "Access-Control-Allow-Headers": "Content-Type",
}


def _make_app(peripheral: BLEPeripheral) -> web.Application:
    async def handle_cmd_vel(request: web.Request) -> web.Response:
        try:
            data = await request.json()
            lx = float(data.get("lx", 0.0))
            az = float(data.get("az", 0.0))
        except (json.JSONDecodeError, ValueError, TypeError):
            return web.json_response(
                {"error": "expected JSON {lx, az}"}, status=400, headers=_CORS_HEADERS
            )
        ok = peripheral.notify_cmd_vel(lx, az)
        return web.json_response({"ok": ok}, headers=_CORS_HEADERS)

    async def handle_e_stop(request: web.Request) -> web.Response:
        try:
            data = await request.json()
            stop = bool(data.get("stop", True))
        except (json.JSONDecodeError, ValueError, TypeError):
            return web.json_response(
                {"error": "expected JSON {stop}"}, status=400, headers=_CORS_HEADERS
            )
        ok = peripheral.notify_e_stop(stop)
        return web.json_response({"ok": ok}, headers=_CORS_HEADERS)

    async def handle_status(_request: web.Request) -> web.Response:
        return web.json_response(
            {
                "connected": peripheral.connected,
                "latency_ms": round(peripheral.latency_ms, 2),
                "writes_per_sec": round(peripheral.writes_per_sec, 1),
            },
            headers=_CORS_HEADERS,
        )

    async def handle_options(_request: web.Request) -> web.Response:
        return web.Response(status=204, headers=_CORS_HEADERS)

    app = web.Application()
    app.router.add_post("/cmd_vel", handle_cmd_vel)
    app.router.add_post("/e_stop", handle_e_stop)
    app.router.add_get("/status", handle_status)
    app.router.add_route("OPTIONS", "/cmd_vel", handle_options)
    app.router.add_route("OPTIONS", "/e_stop", handle_options)
    return app


async def _main() -> None:
    peripheral = BLEPeripheral()
    logger.info("Starting BLE peripheral server...")
    await peripheral.start()

    heartbeat_task = asyncio.create_task(peripheral.heartbeat_loop())

    app = _make_app(peripheral)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "127.0.0.1", HTTP_PORT)
    await site.start()
    logger.info(f"HTTP server listening on http://127.0.0.1:{HTTP_PORT}")
    logger.info("Endpoints: POST /cmd_vel, POST /e_stop, GET /status")

    try:
        while True:
            await asyncio.sleep(3600)
    except asyncio.CancelledError:
        pass
    finally:
        heartbeat_task.cancel()
        await runner.cleanup()


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    )
    try:
        asyncio.run(_main())
    except KeyboardInterrupt:
        logger.info("BLE peripheral shutting down")


if __name__ == "__main__":
    main()
