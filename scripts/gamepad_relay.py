# /// script
# requires-python = ">=3.10"
# dependencies = ["pygame>=2.5", "aiohttp>=3.9"]
# ///
"""Native Mac gamepad reader → BLE relay HTTP endpoint + live viz.

Uses SDL's GameController API (via pygame._sdl2.controller), which applies
SDL_GameControllerDB mappings. This gives the same standard layout the
Chrome Gamepad API uses (axes 0/1 = left stick, 2/3 = right stick, buttons
A=0, B=1, ...) for any controller in the database — no per-controller
configuration needed.

Bypasses the Foxglove panel, which gets throttled by Chromium when the lid
is closed (renderer backgrounding / occlusion). A native process keeps
reading HID input as long as the system is awake.

Endpoints (served on :4201):
    GET /        → live HTML visualizer (open in any browser)
    GET /state   → JSON snapshot of current axes / buttons / cmd_vel

Posts cmd_vel and e_stop to ble_relay.py at http://127.0.0.1:4200.

Run alongside ble_relay.py, or use scripts/teleop_mac.sh to supervise both.
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import os
import signal
import sys
import time

import aiohttp
from aiohttp import web

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
# Allow joystick events even when no window has focus (clamshell mode).
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")

import pygame  # noqa: E402
from pygame._sdl2 import controller as sdl_controller  # noqa: E402

logger = logging.getLogger("gamepad_relay")

DEFAULT_BASE_URL = "http://127.0.0.1:4200"
DEFAULT_VIZ_PORT = 4201
DEFAULT_DEADZONE = 0.15
DEFAULT_MAX_LINEAR = 1.0
DEFAULT_MAX_ANGULAR = 1.5
DEFAULT_RATE_HZ = 50

# SDL_CONTROLLER_AXIS_* — same order as Chrome's "standard" Gamepad mapping
# for sticks (axes 0..3). Triggers are exposed as axes here too.
AXIS_LEFT_X = 0
AXIS_LEFT_Y = 1
AXIS_RIGHT_X = 2
AXIS_RIGHT_Y = 3
AXIS_TRIGGER_LEFT = 4
AXIS_TRIGGER_RIGHT = 5
N_AXES = 6

# SDL_CONTROLLER_BUTTON_* — A/B/X/Y first, then BACK/GUIDE/START/L3/R3/LB/RB/DPad.
BUTTON_A = 0
BUTTON_B = 1
BUTTON_X = 2
BUTTON_Y = 3
BUTTON_BACK = 4
BUTTON_GUIDE = 5
BUTTON_START = 6
BUTTON_LSTICK = 7
BUTTON_RSTICK = 8
BUTTON_LSHOULDER = 9
BUTTON_RSHOULDER = 10
BUTTON_DPAD_UP = 11
BUTTON_DPAD_DOWN = 12
BUTTON_DPAD_LEFT = 13
BUTTON_DPAD_RIGHT = 14
N_BUTTONS = 15

BUTTON_NAMES = [
    "A", "B", "X", "Y", "Back", "Guide", "Start",
    "L3", "R3", "LB", "RB", "DUp", "DDown", "DLeft", "DRight",
]


def _apply_deadzone(v: float, dz: float) -> float:
    if abs(v) < dz:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    return sign * ((abs(v) - dz) / (1.0 - dz))


def _norm(raw_int16: int) -> float:
    # SDL returns axes in int16 range. Negative max is -32768, positive 32767.
    return max(-1.0, min(1.0, raw_int16 / 32767.0))


class GamepadRelay:
    def __init__(
        self,
        base_url: str,
        deadzone: float,
        max_linear: float,
        max_angular: float,
        rate_hz: int,
        invert_linear: bool,
        invert_angular: bool,
        button_arm: int,
        button_estop: int,
        dry_run: bool = False,
    ) -> None:
        self.base_url = base_url.rstrip("/")
        self.deadzone = deadzone
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.period = 1.0 / rate_hz
        self.button_arm = button_arm
        self.button_estop = button_estop
        self.dry_run = dry_run
        # Foxglove panel uses linear = -leftY, angular = -rightX (Gamepad API
        # has Y down = +1 / right = +1). SDL GameController matches.
        self.linear_sign = 1.0 if invert_linear else -1.0
        self.angular_sign = 1.0 if invert_angular else -1.0

        self._ctrl: sdl_controller.Controller | None = None
        self._prev_buttons: list[bool] = [False] * N_BUTTONS

        # Snapshot for the viz / JSON endpoint.
        self.state: dict = {
            "controller": None,
            "connected": False,
            "axes_raw": [0.0] * N_AXES,
            "axes": [0.0] * N_AXES,  # post-deadzone, [-1, 1]
            "buttons": [False] * N_BUTTONS,
            "button_names": BUTTON_NAMES,
            "cmd_vel": {"lx": 0.0, "az": 0.0},
            "max_linear": max_linear,
            "max_angular": max_angular,
            "deadzone": deadzone,
            "ble_connected": None,
            "ble_rtt_ms": None,
            "ble_writes_per_sec": None,
            "post_failures": 0,
            "e_stopped": True,
            "ts": time.time(),
        }
        self._published_zero = True

    def _find_controller(self) -> sdl_controller.Controller | None:
        sdl_controller.quit()
        sdl_controller.init()
        # SDL needs a couple of event-pump cycles before HID enumeration
        # populates on macOS.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            pygame.event.pump()
            if sdl_controller.get_count() > 0:
                break
            time.sleep(0.1)
        if sdl_controller.get_count() == 0:
            return None
        c = sdl_controller.Controller(0)
        logger.info("controller: %s", c.name)
        return c

    async def _post(
        self, session: aiohttp.ClientSession, path: str, payload: dict,
    ) -> bool:
        if self.dry_run:
            logger.info("dry-run POST %s %s", path, payload)
            return True
        url = f"{self.base_url}{path}"
        try:
            async with session.post(
                url, json=payload, timeout=aiohttp.ClientTimeout(total=0.25),
            ) as resp:
                if resp.status >= 400:
                    logger.warning("POST %s → %s", path, resp.status)
                    return False
                return True
        except (aiohttp.ClientError, asyncio.TimeoutError) as exc:
            logger.debug("POST %s failed: %s", path, exc)
            self.state["post_failures"] += 1
            return False

    async def _refresh_ble_status(self, session: aiohttp.ClientSession) -> None:
        url = f"{self.base_url}/status"
        while True:
            try:
                async with session.get(
                    url, timeout=aiohttp.ClientTimeout(total=0.5),
                ) as resp:
                    if resp.status == 200:
                        body = await resp.json()
                        self.state["ble_connected"] = body.get("connected")
                        self.state["ble_rtt_ms"] = body.get("rtt_ms")
                        self.state["ble_writes_per_sec"] = body.get("writes_per_sec")
            except (aiohttp.ClientError, asyncio.TimeoutError):
                self.state["ble_connected"] = False
            await asyncio.sleep(1.0)

    async def run(self, session: aiohttp.ClientSession) -> None:
        backoff = 1.0
        while True:
            self._ctrl = self._find_controller()
            if self._ctrl is None:
                self.state["connected"] = False
                self.state["controller"] = None
                logger.warning("no controller detected — retrying in %.1fs", backoff)
                await asyncio.sleep(backoff)
                backoff = min(backoff * 1.5, 5.0)
                continue
            backoff = 1.0
            self._prev_buttons = [False] * N_BUTTONS
            self.state["connected"] = True
            self.state["controller"] = self._ctrl.name
            try:
                await self._poll_loop(session)
            except pygame.error as exc:
                logger.warning("controller error: %s", exc)
                self.state["connected"] = False

    async def _poll_loop(self, session: aiohttp.ClientSession) -> None:
        assert self._ctrl is not None
        next_tick = time.monotonic()
        while True:
            pygame.event.pump()

            axes_raw = [_norm(self._ctrl.get_axis(k)) for k in range(N_AXES)]
            buttons = [bool(self._ctrl.get_button(k)) for k in range(N_BUTTONS)]
            axes_dz = [_apply_deadzone(v, self.deadzone) for v in axes_raw]

            if 0 <= self.button_arm < N_BUTTONS and buttons[self.button_arm] and not self._prev_buttons[self.button_arm]:
                logger.info("%s pressed → /e_stop {stop:false}", BUTTON_NAMES[self.button_arm])
                await self._post(session, "/e_stop", {"stop": False})
                self.state["e_stopped"] = False
            if 0 <= self.button_estop < N_BUTTONS and buttons[self.button_estop] and not self._prev_buttons[self.button_estop]:
                logger.info("%s pressed → /e_stop {stop:true}", BUTTON_NAMES[self.button_estop])
                await self._post(session, "/e_stop", {"stop": True})
                self.state["e_stopped"] = True
            self._prev_buttons = buttons

            ly = axes_dz[AXIS_LEFT_Y]
            lx = axes_dz[AXIS_LEFT_X]
            cmd_lx = self.linear_sign * ly * self.max_linear
            cmd_az = self.angular_sign * lx * self.max_angular

            has_input = ly != 0.0 or lx != 0.0
            if has_input:
                await self._post(session, "/cmd_vel", {"lx": cmd_lx, "az": cmd_az})
                self.state["cmd_vel"] = {"lx": cmd_lx, "az": cmd_az}
                self._published_zero = False
            elif not self._published_zero:
                await self._post(session, "/cmd_vel", {"lx": 0.0, "az": 0.0})
                self.state["cmd_vel"] = {"lx": 0.0, "az": 0.0}
                self._published_zero = True

            self.state["axes_raw"] = axes_raw
            self.state["axes"] = axes_dz
            self.state["buttons"] = buttons
            self.state["ts"] = time.time()

            next_tick += self.period
            sleep_for = next_tick - time.monotonic()
            if sleep_for < 0:
                next_tick = time.monotonic()
            else:
                await asyncio.sleep(sleep_for)


# ── Web viz ───────────────────────────────────────────────────────────────────


VIZ_HTML = """<!doctype html>
<meta charset="utf-8">
<title>CouchVision Gamepad Relay</title>
<style>
  :root { color-scheme: dark; }
  body {
    font-family: -apple-system, system-ui, sans-serif;
    background: #0b0d10; color: #e6e8eb; margin: 0; padding: 24px;
  }
  h1 { margin: 0 0 4px 0; font-size: 22px; }
  .sub { color: #8a93a0; font-size: 13px; margin-bottom: 20px; }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 24px; max-width: 980px; }
  .card {
    background: #14171c; border: 1px solid #232831; border-radius: 8px;
    padding: 16px;
  }
  .card h2 { margin: 0 0 12px 0; font-size: 14px; letter-spacing: .04em;
             text-transform: uppercase; color: #8a93a0; }
  .stick { display: flex; align-items: center; gap: 16px; margin-bottom: 12px; }
  .stick-label { font-size: 12px; color: #8a93a0; min-width: 64px; }
  .stick-vals { font-variant-numeric: tabular-nums; font-size: 13px;
                color: #c4cad3; min-width: 130px; }
  canvas { background: #1b1f26; border-radius: 50%; }
  .buttons { display: grid; grid-template-columns: repeat(5, 1fr); gap: 6px; }
  .btn {
    background: #1b1f26; border: 1px solid #2a3038; border-radius: 6px;
    padding: 8px 4px; text-align: center; font-size: 12px; color: #8a93a0;
    transition: background .05s, color .05s;
  }
  .btn.on { background: #2563eb; color: white; border-color: #3b82f6; }
  .row { display: flex; justify-content: space-between; padding: 4px 0;
         font-size: 13px; border-bottom: 1px solid #1b1f26; }
  .row:last-child { border-bottom: none; }
  .row .v { font-variant-numeric: tabular-nums; color: #c4cad3; }
  .ok { color: #22c55e; }
  .bad { color: #ef4444; }
  .triggers { display: flex; gap: 12px; margin-top: 8px; }
  .trigger { flex: 1; }
  .bar { height: 8px; background: #1b1f26; border-radius: 4px; overflow: hidden; }
  .bar > i { display: block; height: 100%; background: #3b82f6; width: 0; }
  .full { grid-column: 1 / -1; }
</style>
<h1>CouchVision Gamepad Relay</h1>
<div class="sub" id="sub">connecting…</div>
<div class="grid">
  <div class="card">
    <h2>Sticks</h2>
    <div class="stick">
      <div class="stick-label">LEFT</div>
      <canvas id="lstick" width="120" height="120"></canvas>
      <div class="stick-vals" id="lvals">x=0.000<br>y=0.000</div>
    </div>
    <div class="stick">
      <div class="stick-label">RIGHT</div>
      <canvas id="rstick" width="120" height="120"></canvas>
      <div class="stick-vals" id="rvals">x=0.000<br>y=0.000</div>
    </div>
    <div class="triggers">
      <div class="trigger">
        <div class="stick-label">L2</div>
        <div class="bar"><i id="lt"></i></div>
      </div>
      <div class="trigger">
        <div class="stick-label">R2</div>
        <div class="bar"><i id="rt"></i></div>
      </div>
    </div>
  </div>
  <div class="card">
    <h2>Buttons</h2>
    <div class="buttons" id="buttons"></div>
  </div>
  <div class="card full">
    <h2>Outgoing cmd_vel + BLE</h2>
    <div class="row"><span>linear.x</span><span class="v" id="lx">0.000 m/s</span></div>
    <div class="row"><span>angular.z</span><span class="v" id="az">0.000 rad/s</span></div>
    <div class="row"><span>BLE connected</span><span class="v" id="ble">—</span></div>
    <div class="row"><span>BLE RTT</span><span class="v" id="rtt">—</span></div>
    <div class="row"><span>writes / sec</span><span class="v" id="wps">—</span></div>
    <div class="row"><span>POST failures</span><span class="v" id="fail">0</span></div>
  </div>
</div>
<script>
const $ = (id) => document.getElementById(id);

function drawStick(canvas, x, y) {
  const ctx = canvas.getContext("2d");
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0, 0, w, h);
  ctx.strokeStyle = "#2a3038"; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.arc(w/2, h/2, w/2 - 2, 0, Math.PI*2); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(w/2, 4); ctx.lineTo(w/2, h-4); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(4, h/2); ctx.lineTo(w-4, h/2); ctx.stroke();
  const cx = w/2 + x * (w/2 - 8);
  const cy = h/2 + y * (h/2 - 8);
  ctx.fillStyle = "#3b82f6";
  ctx.beginPath(); ctx.arc(cx, cy, 8, 0, Math.PI*2); ctx.fill();
}

function buildButtons(names) {
  const grid = $("buttons");
  grid.innerHTML = "";
  for (let i = 0; i < names.length; i++) {
    const el = document.createElement("div");
    el.className = "btn";
    el.id = "b" + i;
    el.textContent = names[i];
    grid.appendChild(el);
  }
}

let buttonsReady = false;

async function tick() {
  try {
    const r = await fetch("/state");
    const s = await r.json();
    $("sub").textContent = s.connected
      ? `${s.controller}  •  deadzone ${s.deadzone}  •  max ${s.max_linear} m/s / ${s.max_angular} rad/s`
      : "no controller detected";

    if (!buttonsReady && s.button_names) { buildButtons(s.button_names); buttonsReady = true; }

    const a = s.axes || [];
    drawStick($("lstick"), a[0] || 0, a[1] || 0);
    drawStick($("rstick"), a[2] || 0, a[3] || 0);
    $("lvals").innerHTML = `x=${(a[0]||0).toFixed(3)}<br>y=${(a[1]||0).toFixed(3)}`;
    $("rvals").innerHTML = `x=${(a[2]||0).toFixed(3)}<br>y=${(a[3]||0).toFixed(3)}`;
    $("lt").style.width = (Math.max(0, (a[4]||0) + 1) / 2 * 100) + "%";
    $("rt").style.width = (Math.max(0, (a[5]||0) + 1) / 2 * 100) + "%";

    (s.buttons || []).forEach((on, i) => {
      const el = document.getElementById("b" + i);
      if (el) el.classList.toggle("on", on);
    });

    $("lx").textContent = (s.cmd_vel?.lx || 0).toFixed(3) + " m/s";
    $("az").textContent = (s.cmd_vel?.az || 0).toFixed(3) + " rad/s";
    const bc = s.ble_connected;
    $("ble").innerHTML = bc === true
      ? '<span class="ok">yes</span>'
      : bc === false ? '<span class="bad">no</span>' : "—";
    $("rtt").textContent = s.ble_rtt_ms != null ? s.ble_rtt_ms.toFixed(1) + " ms" : "—";
    $("wps").textContent = s.ble_writes_per_sec != null ? s.ble_writes_per_sec.toFixed(1) : "—";
    $("fail").textContent = s.post_failures || 0;
  } catch (e) {
    $("sub").textContent = "lost connection to relay";
  }
}
setInterval(tick, 50);
tick();
</script>
"""


def _cors_headers(_req):
    return {
        "Access-Control-Allow-Origin": "*",
        "Access-Control-Allow-Methods": "GET, POST, OPTIONS",
        "Access-Control-Allow-Headers": "Content-Type",
    }


def _make_viz_app(relay: GamepadRelay) -> web.Application:
    async def index(_req):
        return web.Response(text=VIZ_HTML, content_type="text/html")

    async def state(req):
        return web.json_response(relay.state, headers=_cors_headers(req))

    async def handle_e_stop(req):
        body = await req.json()
        stop = bool(body.get("stop", True))
        relay.state["e_stopped"] = stop
        return web.json_response({"ok": True}, headers=_cors_headers(req))

    async def cors_preflight(req):
        return web.Response(headers=_cors_headers(req))

    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_get("/state", state)
    app.router.add_post("/e_stop", handle_e_stop)
    app.router.add_route("OPTIONS", "/state", cors_preflight)
    app.router.add_route("OPTIONS", "/e_stop", cors_preflight)
    return app


# ── CLI ───────────────────────────────────────────────────────────────────────


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--base-url", default=DEFAULT_BASE_URL,
                   help="ble_relay HTTP endpoint")
    p.add_argument("--viz-port", type=int, default=DEFAULT_VIZ_PORT,
                   help="port for the live HTML/JSON viz")
    p.add_argument("--deadzone", type=float, default=DEFAULT_DEADZONE)
    p.add_argument("--max-linear", type=float, default=DEFAULT_MAX_LINEAR)
    p.add_argument("--max-angular", type=float, default=DEFAULT_MAX_ANGULAR)
    p.add_argument("--rate", type=int, default=DEFAULT_RATE_HZ)
    p.add_argument("--button-arm", type=int, default=BUTTON_A,
                   help="button index to publish e_stop {stop:false} (default A=0)")
    p.add_argument("--button-estop", type=int, default=BUTTON_B,
                   help="button index to publish e_stop {stop:true} (default B=1)")
    p.add_argument("--invert-linear", action="store_true")
    p.add_argument("--invert-angular", action="store_true")
    p.add_argument("--dry-run", action="store_true",
                   help="log cmd_vel to console instead of POSTing to BLE relay")
    p.add_argument("--list", action="store_true",
                   help="list detected controllers and exit")
    p.add_argument("-v", "--verbose", action="store_true")
    return p.parse_args()


def _list_controllers() -> int:
    pygame.init()
    sdl_controller.init()
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and sdl_controller.get_count() == 0:
        pygame.event.pump()
        time.sleep(0.1)
    n = sdl_controller.get_count()
    if n == 0:
        print("no controllers detected")
        # Fall back: show raw joysticks for diagnostics.
        pygame.joystick.init()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and pygame.joystick.get_count() == 0:
            pygame.event.pump()
            time.sleep(0.1)
        nj = pygame.joystick.get_count()
        if nj > 0:
            print(f"(but {nj} raw HID joystick(s) visible — not in SDL gamecontrollerdb)")
            for k in range(nj):
                j = pygame.joystick.Joystick(k); j.init()
                print(f"  [{k}] {j.get_name()} guid={j.get_guid()}")
        return 1
    for i in range(n):
        c = sdl_controller.Controller(i)
        print(f"[{i}] {c.name}  (SDL standard mapping)")
    return 0


async def _async_main(args: argparse.Namespace) -> None:
    pygame.init()
    sdl_controller.init()

    relay = GamepadRelay(
        base_url=args.base_url,
        deadzone=args.deadzone,
        max_linear=args.max_linear,
        max_angular=args.max_angular,
        rate_hz=args.rate,
        invert_linear=args.invert_linear,
        invert_angular=args.invert_angular,
        button_arm=args.button_arm,
        button_estop=args.button_estop,
        dry_run=args.dry_run,
    )

    app = _make_viz_app(relay)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", args.viz_port)
    await site.start()
    logger.info("viz: http://0.0.0.0:%d/", args.viz_port)

    async with aiohttp.ClientSession() as session:
        status_task = None
        if not args.dry_run:
            status_task = asyncio.create_task(relay._refresh_ble_status(session))
        try:
            await relay.run(session)
        finally:
            if status_task is not None:
                status_task.cancel()
            await runner.cleanup()


def main() -> None:
    args = _parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    )
    if args.list:
        sys.exit(_list_controllers())

    loop = asyncio.new_event_loop()

    def _shutdown(*_a):
        for t in asyncio.all_tasks(loop):
            t.cancel()

    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, _shutdown)

    try:
        loop.run_until_complete(_async_main(args))
    except asyncio.CancelledError:
        pass
    finally:
        pygame.quit()
        loop.close()


if __name__ == "__main__":
    main()
