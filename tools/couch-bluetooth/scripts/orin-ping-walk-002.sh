#!/usr/bin/env bash
set -euo pipefail

ROOT="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if [[ -z "${IN_NIX_SHELL:-}" && "${SKIP_NIX:-0}" != "1" ]]; then
  exec nix develop -c "$0" "$@"
fi

TARGET="${TARGET:-charlie-jetson-orin-nano.tail0eb43d.ts.net}"
OUT="${OUT:-runs/ping-walk-002}"
DISTANCES="${DISTANCES:-0,5,10}"
HOLD_SECONDS="${HOLD_SECONDS:-30}"
SAMPLE_RATE_HZ="${SAMPLE_RATE_HZ:-2}"

if [[ "${FORCE:-0}" != "1" && -e "$OUT/raw/samples.csv" ]]; then
  echo "Refusing to overwrite existing $OUT/raw/samples.csv" >&2
  echo "Use FORCE=1 $0 if you intentionally want to replace run 002." >&2
  exit 1
fi

force_args=()
if [[ "${FORCE:-0}" == "1" ]]; then
  force_args=(--force)
fi

echo "Preflight: Tailscale ping $TARGET"
tailscale ping --timeout=3s --c=3 "$TARGET"

echo
echo "Collecting $OUT at distances $DISTANCES."
echo "At each prompt: stand at the marked distance, press Enter, and hold still until the station finishes."
echo

./couch-range ping-walk \
  --out "$OUT" \
  --target "$TARGET" \
  --distances "$DISTANCES" \
  --hold-seconds "$HOLD_SECONDS" \
  --sample-rate-hz "$SAMPLE_RATE_HZ" \
  "${force_args[@]}"

./couch-range analyze "$OUT"
./couch-range report "$OUT"

echo
echo "Done."
echo "Report: $OUT/report.md"
echo "Charts: $OUT/charts/"
