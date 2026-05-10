# /// script
# requires-python = ">=3.10"
# dependencies = ["pygame>=2.5", "aiohttp>=3.9"]
# ///
"""Native Mac gamepad reader → BLE relay HTTP endpoint.

Bypasses the Foxglove panel, which gets throttled by Chromium when the lid
is closed (renderer backgrounding / occlusion). A native process keeps
reading HID input as long as Bluetooth is up and the system is awake.

Flow:
    gamepad (HID) → pygame → POST localhost:4200/cmd_vel → ble_relay.py → BLE
                                  → POST localhost:4200/e_stop      → BLE

Run alongside ble_relay.py:
    uv run scripts/ble_relay.py        # in one terminal
    uv run scripts/gamepad_relay.py    # in another

Mapping (Xbox/PS-style "standard" controller on macOS):
    left stick Y  (axis 1)   → linear.x  (forward = stick up = -axis)
    right stick X (axis 2)   → angular.z (CCW = stick left = -axis)
    button 0      (A / X)    → arm     (POST /e_stop {stop:false})
    button 1      (B / O)    → e-stop  (POST /e_stop {stop:true})

Defaults match the Foxglove HardwareSafetyPanel: deadzone 0.15,
max_linear 0.5 m/s, max_angular 2.0 rad/s, 50 Hz publish rate.
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

# Don't force SDL_VIDEODRIVER=dummy — on macOS that disables the run-loop
# sources SDL needs for HID joystick detection. pygame.init() does not pop
# up a window unless we call pygame.display.set_mode().
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

import pygame  # noqa: E402

logger = logging.getLogger("gamepad_relay")

DEFAULT_BASE_URL = "http://127.0.0.1:4200"
DEFAULT_DEADZONE = 0.15
DEFAULT_MAX_LINEAR = 0.5
DEFAULT_MAX_ANGULAR = 2.0
DEFAULT_RATE_HZ = 50

# Standard mapping (matches Gamepad API "standard" — used by Foxglove panel).
AXIS_LEFT_Y = 1
AXIS_RIGHT_X = 2
BUTTON_ARM = 0    # A / Cross
BUTTON_ESTOP = 1  # B / Circle


def _apply_deadzone(v: float, dz: float) -> float:
    if abs(v) < dz:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    return sign * ((abs(v) - dz) / (1.0 - dz))


class GamepadRelay:
    def __init__(
        self,
        base_url: str,
        deadzone: float,
        max_linear: float,
        max_angular: float,
        rate_hz: int,
        axis_left_y: int,
        axis_right_x: int,
        button_arm: int,
        button_estop: int,
        invert_linear: bool,
        invert_angular: bool,
    ) -> None:
        self.base_url = base_url.rstrip("/")
        self.deadzone = deadzone
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.period = 1.0 / rate_hz
        self.axis_left_y = axis_left_y
        self.axis_right_x = axis_right_x
        self.button_arm = button_arm
        self.button_estop = button_estop
        # Panel does linear = -leftY, angular = -rightX. Mirror that.
        self.linear_sign = 1.0 if invert_linear else -1.0
        self.angular_sign = 1.0 if invert_angular else -1.0

        self._joy: pygame.joystick.JoystickType | None = None
        self._prev_buttons: list[bool] = []
        self._last_lx = 0.0
        self._last_az = 0.0
        self._published_zero = True

    def _find_joystick(self) -> pygame.joystick.JoystickType | None:
        pygame.joystick.quit()
        pygame.joystick.init()
        # SDL on macOS needs several event-pump cycles before HID enumeration
        # actually populates. Without this we'd report "no gamepads" even
        # when the controller is plugged in and visible to IOKit.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            pygame.event.pump()
            if pygame.joystick.get_count() > 0:
                break
            time.sleep(0.1)
        if pygame.joystick.get_count() == 0:
            return None
        joy = pygame.joystick.Joystick(0)
        joy.init()
        logger.info(
            "gamepad: %s (axes=%d buttons=%d)",
            joy.get_name(),
            joy.get_numaxes(),
            joy.get_numbuttons(),
        )
        return joy

    async def _post(self, session: aiohttp.ClientSession, path: str, payload: dict) -> None:
        url = f"{self.base_url}{path}"
        try:
            async with session.post(url, json=payload, timeout=aiohttp.ClientTimeout(total=0.25)) as resp:
                if resp.status >= 400:
                    logger.warning("POST %s → %s", path, resp.status)
        except (aiohttp.ClientError, asyncio.TimeoutError) as exc:
            logger.debug("POST %s failed: %s", path, exc)

    async def run(self) -> None:
        pygame.init()
        pygame.joystick.init()

        async with aiohttp.ClientSession() as session:
            backoff = 1.0
            while True:
                self._joy = self._find_joystick()
                if self._joy is None:
                    logger.warning("no gamepad detected — retrying in %.1fs", backoff)
                    await asyncio.sleep(backoff)
                    backoff = min(backoff * 1.5, 5.0)
                    continue
                backoff = 1.0
                self._prev_buttons = [False] * self._joy.get_numbuttons()
                try:
                    await self._poll_loop(session)
                except pygame.error as exc:
                    logger.warning("pygame error (controller likely disconnected): %s", exc)
                    # Loop back to re-detect.

    async def _poll_loop(self, session: aiohttp.ClientSession) -> None:
        assert self._joy is not None
        next_tick = time.monotonic()
        while True:
            pygame.event.pump()

            n_buttons = self._joy.get_numbuttons()
            buttons = [bool(self._joy.get_button(i)) for i in range(n_buttons)]

            # Edge-triggered arm / e-stop.
            if self._edge(buttons, self.button_arm):
                logger.info("button %d (arm) pressed → /e_stop {stop:false}", self.button_arm)
                await self._post(session, "/e_stop", {"stop": False})
            if self._edge(buttons, self.button_estop):
                logger.info("button %d (e-stop) pressed → /e_stop {stop:true}", self.button_estop)
                await self._post(session, "/e_stop", {"stop": True})
            self._prev_buttons = buttons

            n_axes = self._joy.get_numaxes()
            left_y = self._joy.get_axis(self.axis_left_y) if n_axes > self.axis_left_y else 0.0
            right_x = self._joy.get_axis(self.axis_right_x) if n_axes > self.axis_right_x else 0.0

            ly = _apply_deadzone(left_y, self.deadzone)
            rx = _apply_deadzone(right_x, self.deadzone)

            lx = self.linear_sign * ly * self.max_linear
            az = self.angular_sign * rx * self.max_angular

            has_input = ly != 0.0 or rx != 0.0
            if has_input:
                await self._post(session, "/cmd_vel", {"lx": lx, "az": az})
                self._last_lx, self._last_az = lx, az
                self._published_zero = False
            elif not self._published_zero:
                await self._post(session, "/cmd_vel", {"lx": 0.0, "az": 0.0})
                self._last_lx = self._last_az = 0.0
                self._published_zero = True

            next_tick += self.period
            sleep_for = next_tick - time.monotonic()
            if sleep_for < 0:
                next_tick = time.monotonic()
            else:
                await asyncio.sleep(sleep_for)

    def _edge(self, buttons: list[bool], idx: int) -> bool:
        if idx >= len(buttons) or idx >= len(self._prev_buttons):
            return False
        return buttons[idx] and not self._prev_buttons[idx]


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--base-url", default=DEFAULT_BASE_URL)
    p.add_argument("--deadzone", type=float, default=DEFAULT_DEADZONE)
    p.add_argument("--max-linear", type=float, default=DEFAULT_MAX_LINEAR)
    p.add_argument("--max-angular", type=float, default=DEFAULT_MAX_ANGULAR)
    p.add_argument("--rate", type=int, default=DEFAULT_RATE_HZ, help="publish rate in Hz")
    p.add_argument("--axis-left-y", type=int, default=AXIS_LEFT_Y)
    p.add_argument("--axis-right-x", type=int, default=AXIS_RIGHT_X)
    p.add_argument("--button-arm", type=int, default=BUTTON_ARM)
    p.add_argument("--button-estop", type=int, default=BUTTON_ESTOP)
    p.add_argument("--invert-linear", action="store_true")
    p.add_argument("--invert-angular", action="store_true")
    p.add_argument("--list", action="store_true", help="list detected gamepads and exit")
    p.add_argument("-v", "--verbose", action="store_true")
    return p.parse_args()


def _list_gamepads() -> int:
    pygame.init()
    pygame.joystick.init()
    # Pump events until enumeration settles.
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and pygame.joystick.get_count() == 0:
        pygame.event.pump()
        time.sleep(0.1)
    n = pygame.joystick.get_count()
    if n == 0:
        print("no gamepads detected")
        return 1
    for i in range(n):
        j = pygame.joystick.Joystick(i)
        j.init()
        print(f"[{i}] {j.get_name()}  axes={j.get_numaxes()}  buttons={j.get_numbuttons()}  hats={j.get_numhats()}")
    return 0


def main() -> None:
    args = _parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    )

    if args.list:
        sys.exit(_list_gamepads())

    relay = GamepadRelay(
        base_url=args.base_url,
        deadzone=args.deadzone,
        max_linear=args.max_linear,
        max_angular=args.max_angular,
        rate_hz=args.rate,
        axis_left_y=args.axis_left_y,
        axis_right_x=args.axis_right_x,
        button_arm=args.button_arm,
        button_estop=args.button_estop,
        invert_linear=args.invert_linear,
        invert_angular=args.invert_angular,
    )

    loop = asyncio.new_event_loop()

    def _shutdown(*_args: object) -> None:
        for task in asyncio.all_tasks(loop):
            task.cancel()

    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, _shutdown)

    try:
        loop.run_until_complete(relay.run())
    except asyncio.CancelledError:
        pass
    finally:
        pygame.quit()
        loop.close()


if __name__ == "__main__":
    main()
