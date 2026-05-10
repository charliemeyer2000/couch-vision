#!/usr/bin/env bash
# One-command Mac-side teleop:
#   - BLE peripheral / HTTP relay on :4200 (talks to Jetson)
#   - Native gamepad reader (pygame) → POSTs /cmd_vel & /e_stop
#
# Run on the Mac after `make full-stack VESC=1` is up on the Jetson.
# Ctrl-C tears both processes down.

set -euo pipefail

cd "$(dirname "$0")/.."

BLE_LOG=/tmp/couchvision-ble-relay.log
GP_LOG=/tmp/couchvision-gamepad-relay.log

cleanup() {
  echo
  echo "==> shutting down teleop"
  if [[ -n "${BLE_PID:-}" ]]; then kill "$BLE_PID" 2>/dev/null || true; fi
  if [[ -n "${GP_PID:-}" ]]; then kill "$GP_PID" 2>/dev/null || true; fi
  wait 2>/dev/null || true
}
trap cleanup INT TERM EXIT

echo "==> starting BLE relay  (logs: $BLE_LOG)"
uv run scripts/ble_relay.py >"$BLE_LOG" 2>&1 &
BLE_PID=$!

# Wait (up to 15s) for ble_relay's HTTP endpoint to come up.
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

echo "==> starting gamepad relay  (logs: $GP_LOG)"
uv run scripts/gamepad_relay.py "$@" >"$GP_LOG" 2>&1 &
GP_PID=$!

echo "==> teleop running. tail logs in another shell:"
echo "    tail -f $BLE_LOG"
echo "    tail -f $GP_LOG"
echo "==> Ctrl-C to stop."

# Exit (and tear down via trap) as soon as either child dies.
wait -n "$BLE_PID" "$GP_PID"
