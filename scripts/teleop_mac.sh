#!/usr/bin/env bash
# One-command Mac-side teleop (clamshell-safe):
#   - Prevents macOS sleep so the Mac keeps running with the lid closed
#   - BLE peripheral / HTTP relay on :4200 (talks to Jetson)
#   - Native gamepad reader (pygame/SDL) → POSTs /cmd_vel & /e_stop
#   - Live web viz on :4201 (accessible before you close the lid)
#
# Run on the Mac after `make full-stack VESC=1` is up on the Jetson.
# Ctrl-C tears everything down and restores power settings.
#
# Usage:
#   ./scripts/teleop_mac.sh              # normal: BLE relay + gamepad
#   ./scripts/teleop_mac.sh --dry-run    # test: gamepad only, no BLE needed

set -euo pipefail

cd "$(dirname "$0")/.."

BLE_LOG=/tmp/couchvision-ble-relay.log
GP_LOG=/tmp/couchvision-gamepad-relay.log

cleanup() {
  echo
  echo "==> shutting down teleop"
  [[ -n "${GP_PID:-}" ]] && kill "$GP_PID" 2>/dev/null || true
  [[ -n "${BLE_PID:-}" ]] && kill "$BLE_PID" 2>/dev/null || true
  [[ -n "${CAFFEINATE_PID:-}" ]] && kill "$CAFFEINATE_PID" 2>/dev/null || true
  if [[ "${RESTORE_PMSET:-0}" == "1" ]]; then
    echo "==> restoring power settings"
    sudo pmset -a disablesleep 0 2>/dev/null || true
  fi
  wait 2>/dev/null || true
}
trap cleanup INT TERM EXIT

# ── Prevent macOS sleep (critical for clamshell/backpack mode) ─────
echo "==> preventing macOS sleep (clamshell mode)"
# disablesleep: overrides lid-close sleep entirely
sudo pmset -a disablesleep 1
# battery sleep=0: prevent sleep on battery (nix-darwin only sets AC)
sudo pmset -b sleep 0
RESTORE_PMSET=1
# caffeinate: belt-and-suspenders IOPMAssertions
# -d display  -i idle  -m disk  -s system  -u user-active
caffeinate -dimsu &
CAFFEINATE_PID=$!
echo "    disablesleep=1, caffeinate pid=$CAFFEINATE_PID"

# ── Check for --dry-run (pass-through to gamepad_relay.py) ─────────
DRY_RUN=0
for arg in "$@"; do
  [[ "$arg" == "--dry-run" ]] && DRY_RUN=1
done

# ── BLE relay (skip in dry-run mode) ───────────────────────────────
if [[ "$DRY_RUN" == "0" ]]; then
  echo "==> starting BLE relay  (logs: $BLE_LOG)"
  uv run scripts/ble_relay.py >"$BLE_LOG" 2>&1 &
  BLE_PID=$!

  ready=0
  for _ in {1..60}; do
    if curl -fsS http://127.0.0.1:4200/status >/dev/null 2>&1; then
      ready=1
      break
    fi
    if ! kill -0 "$BLE_PID" 2>/dev/null; then
      echo "ble_relay died — check $BLE_LOG" >&2
      tail -n 30 "$BLE_LOG" >&2 || true
      exit 1
    fi
    sleep 0.25
  done
  if [[ "$ready" != "1" ]]; then
    echo "ble_relay did not come up on :4200 within 15s." >&2
    echo "  - is Bluetooth turned on?  (System Settings → Bluetooth)" >&2
    echo "  - did macOS show a permission prompt the first time?" >&2
    echo "  - last lines of $BLE_LOG:" >&2
    tail -n 20 "$BLE_LOG" >&2 || true
    exit 1
  fi
else
  echo "==> dry-run mode: skipping BLE relay"
fi

# ── Gamepad relay ──────────────────────────────────────────────────
echo "==> starting gamepad relay  (logs: $GP_LOG)"
uv run scripts/gamepad_relay.py "$@" >"$GP_LOG" 2>&1 &
GP_PID=$!

# Brief wait to catch early startup failures
sleep 1
if ! kill -0 "$GP_PID" 2>/dev/null; then
  echo "gamepad_relay died on startup — check $GP_LOG" >&2
  tail -n 20 "$GP_LOG" >&2 || true
  exit 1
fi

echo "==> teleop running (clamshell-safe). Viz: http://127.0.0.1:4201/"
echo "    tail -f $GP_LOG     # gamepad"
[[ "$DRY_RUN" == "0" ]] && echo "    tail -f $BLE_LOG    # BLE"
echo "==> close the lid and go. Ctrl-C to stop."

if [[ "$DRY_RUN" == "0" ]]; then
  wait -n "$BLE_PID" "$GP_PID"
else
  wait "$GP_PID"
fi
