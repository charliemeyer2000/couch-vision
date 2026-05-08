#!/usr/bin/env bash
# Keep Foxglove receiving gamepad input with the lid closed.
#
# Stacks every macOS workaround:
#   1. caffeinate -dimsu  -> blocks display + system + idle sleep
#   2. pmset disablesleep -> overrides lid-close sleep (needs sudo)
#   3. osascript activate -> keeps Foxglove frontmost so Gamepad API polls
#   4. cliclick mouse nudge -> defeats user-idle detection
#
# Usage:  ./scripts/keep-foxglove-alive.sh
# Stop:   Ctrl-C (cleans up caffeinate + restores pmset)

set -euo pipefail

APP_NAME="Foxglove"
TICK_SECONDS=3

if ! pgrep -xq "$APP_NAME"; then
  echo "warn: $APP_NAME is not running — start it first" >&2
fi

if ! command -v cliclick >/dev/null 2>&1; then
  echo "info: cliclick not installed (brew install cliclick) — skipping mouse nudge"
  HAVE_CLICLICK=0
else
  HAVE_CLICLICK=1
fi

echo "==> requesting sudo to disable lid-close sleep"
sudo pmset -a disablesleep 1
RESTORE_PMSET=1

echo "==> starting caffeinate (display + system + idle)"
caffeinate -dimsu &
CAFFEINATE_PID=$!

cleanup() {
  echo
  echo "==> cleaning up"
  kill "$CAFFEINATE_PID" 2>/dev/null || true
  if [[ "${RESTORE_PMSET:-0}" == "1" ]]; then
    sudo pmset -a disablesleep 0 || true
  fi
}
trap cleanup EXIT INT TERM

echo "==> keeping $APP_NAME frontmost (every ${TICK_SECONDS}s). Ctrl-C to stop."
while true; do
  osascript -e "tell application \"$APP_NAME\" to activate" 2>/dev/null || true
  if [[ "$HAVE_CLICLICK" == "1" ]]; then
    # Move mouse 1px right then back — defeats idle detection without disturbing UI
    cliclick m:+1,+0 m:-1,+0 >/dev/null 2>&1 || true
  fi
  sleep "$TICK_SECONDS"
done
