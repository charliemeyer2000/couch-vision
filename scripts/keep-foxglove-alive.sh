#!/usr/bin/env bash
# Keep Foxglove receiving gamepad input with the lid closed.
#
# Stacks every macOS workaround:
#   1. relaunch Foxglove with Chromium flags that disable renderer backgrounding
#      (without these, lid-close occludes the window and Chromium throttles
#      JS timers to ~1Hz, killing Gamepad API polling after ~30s)
#   2. caffeinate -dimsu  -> blocks display + system + idle sleep
#   3. pmset disablesleep -> overrides lid-close sleep (needs sudo)
#   4. osascript activate -> keeps Foxglove frontmost so Gamepad API polls
#   5. cliclick mouse nudge -> defeats user-idle detection
#
# Usage:  ./scripts/keep-foxglove-alive.sh
# Stop:   Ctrl-C (cleans up caffeinate + restores pmset)

set -euo pipefail

APP_NAME="Foxglove"
TICK_SECONDS=3

CHROMIUM_FLAGS=(
  --disable-background-timer-throttling
  --disable-renderer-backgrounding
  --disable-backgrounding-occluded-windows
)

needs_relaunch() {
  # Renderer process carries the same argv flags Electron passed in.
  local pid
  pid=$(pgrep -f "Foxglove Helper \(Renderer\)" | head -n1 || true)
  [[ -z "$pid" ]] && return 0
  local args
  args=$(ps -o command= -p "$pid" 2>/dev/null || true)
  for flag in "${CHROMIUM_FLAGS[@]}"; do
    [[ "$args" == *"$flag"* ]] || return 0
  done
  return 1
}

if pgrep -xq "$APP_NAME" && ! needs_relaunch; then
  echo "==> $APP_NAME already running with anti-throttling flags"
else
  if pgrep -xq "$APP_NAME"; then
    echo "==> quitting $APP_NAME (missing anti-throttling flags)"
    osascript -e "tell application \"$APP_NAME\" to quit" 2>/dev/null || true
    for _ in {1..20}; do
      pgrep -xq "$APP_NAME" || break
      sleep 0.25
    done
    pgrep -xq "$APP_NAME" && pkill -x "$APP_NAME" || true
  fi
  echo "==> launching $APP_NAME with anti-throttling flags"
  open -na "$APP_NAME" --args "${CHROMIUM_FLAGS[@]}"
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
