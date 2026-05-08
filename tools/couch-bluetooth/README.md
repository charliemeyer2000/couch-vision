# Couch Bluetooth Range Test

This tool contains a reproducible field-test kit for deciding whether a laptop
with a wired Xbox controller can safely control a couch over the intended
Bluetooth-backed control path to a Jetson Orin/Nano-class device.

The first pass should use synthetic commands only. Do not move the couch until
the synthetic command path is reliable at the required distance.

## Quick Start

```sh
nix develop
./couch-range inventory --out runs/inventory
./couch-range simulate --out runs/simulated
./couch-range analyze runs/simulated
./couch-range report runs/simulated
```

Open `runs/simulated/report.md` and the PNG charts in `runs/simulated/charts/`.

Reports and charts use the Midnight palette from `midnight.nvim`, with
Berkeley Mono preferred when it is installed and common Matplotlib font
fallbacks otherwise.

Inventory output can include local IP addresses, Tailnet peer details, Bluetooth
MAC addresses, and device names. Review it before sharing a run directory.

## Ping-Only Walk Test

This is the dead-simple first real test. It only measures laptop-to-Jetson
Tailscale ping reachability by distance. It does not use the controller, does
not prove Bluetooth command-path reliability, and does not move the couch.

```sh
cd tools/couch-bluetooth
nix develop
tailscale ping --timeout=3s --c=3 charlie-jetson-orin-nano.tail0eb43d.ts.net
./couch-range ping-walk --out runs/ping-walk-001 --target charlie-jetson-orin-nano.tail0eb43d.ts.net --distances 0,5,10,20,30,50 --hold-seconds 30 --sample-rate-hz 2
./couch-range analyze runs/ping-walk-001
./couch-range report runs/ping-walk-001
```

If the initial `tailscale ping` fails while the laptop is next to the
Jetson/Orin, stop and fix reachability before walking. After the run, open
`runs/ping-walk-001/report.md` and `runs/ping-walk-001/charts/`.

## CouchVision No-Motion Walk Test

This probes the existing CouchVision BLE relay without sending `/cmd_vel` or
`/e_stop`. It polls `GET /status`, records the relay heartbeat RTT when present,
and runs through the same distance charts/report as the other collectors.

Start the CouchVision relay first:

```sh
cd /home/barrett/dev/couch-vision
uv run scripts/ble_relay.py
```

Then collect from this tool directory:

```sh
cd tools/couch-bluetooth
nix develop
./couch-range couchvision-status --relay http://127.0.0.1:4200
./couch-range couchvision-walk --relay http://127.0.0.1:4200 --out runs/couchvision-001 --distances 0,5,10 --hold-seconds 30 --sample-rate-hz 2
./couch-range analyze runs/couchvision-001
./couch-range report runs/couchvision-001
```

An HTTP response alone is logged but not counted as a Jetson-side
acknowledgement. The walk test only counts samples where the relay reports
connected and has a positive heartbeat pong RTT because the current CouchVision
relay does not acknowledge each motion command from the Jetson side.

## Real Walk-Away Test

On the Orin/Nano near the couch:

```sh
nix develop
./couch-range receiver --bind 0.0.0.0 --port 8765
```

On the laptop with the wired controller:

```sh
nix develop
./couch-range inventory --out runs/field-001
./couch-range run --config config.example.toml --out runs/field-001 --target ORIN_HOST_OR_IP --port 8765 --distances 0,5,10,20,30,50 --hold-seconds 30
./couch-range analyze runs/field-001
./couch-range report runs/field-001
```

Replace `ORIN_HOST_OR_IP` with the Orin/Nano address on the transport you are
actually testing. If the goal is Bluetooth range, use the Bluetooth-backed
address/path, not a Wi-Fi-only shortcut.

## Non-Developer Field Workflow

1. Put the laptop next to the Orin/Nano.
2. Plug the Xbox controller into the laptop.
3. Start the receiver on the Orin/Nano.
4. Start the logger on the laptop.
5. Press Enter at 0 m.
6. Walk to 5 m, press Enter, and wait for the timer.
7. Walk to 10 m, press Enter, and wait for the timer.
8. Continue until the link is unreliable or disconnected.
9. Stop the logger with Ctrl-C after the last station finishes.
10. Run the analyze and report commands.
11. Open the generated Markdown report and PNG charts.

Manual distance marks from a tape measure or rangefinder are the preferred
ground truth for short-range tests. GPS and RSSI are recorded as approximations
when available, not as proof of distance.

## Safety

The default thresholds are conservative:

- Reliable: success rate >= 99%, no reconnects, p95 latency <= 100 ms, loss <= 1%.
- Marginal: success rate >= 95%, p95 latency <= 250 ms, loss <= 5%, no sustained disconnects.
- Unsafe: anything worse, repeated disconnects, or command stalls.

Real couch motion should only be tested after synthetic commands and harmless
controller events are reliable. A physical emergency stop or dead-man behavior
should be present before actuation tests.
