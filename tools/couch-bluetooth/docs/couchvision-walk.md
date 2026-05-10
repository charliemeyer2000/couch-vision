# CouchVision Relay Walk Test

This is the first no-motion CouchVision relay walk test. It checks whether the
test laptop can keep reaching the local CouchVision relay while the operator
stands at marked distances.

## What It Measures

- `GET /status` reachability to the local CouchVision relay.
- The CouchVision heartbeat/pong RTT reported by that relay.
- Failure, timeout, and recovery patterns while the laptop is walked away from
  the couch/Jetson setup.

## What It Does Not Measure

- Jetson-side acknowledgement for each command.
- BLE notification receipt for each couch command.
- Controller input, ROS2 `/cmd_vel`, VESC serial control, or couch motion.
- GPS-verified distance. Distance is the operator-declared marker.

## Prerequisites

- `scripts/ble_relay.py` from `couch-vision` is running and serving the relay at
  `http://127.0.0.1:4200`.
- The Jetson `ble_bridge` is connected to the relay before the walk starts.
- Couch motion is disabled or physically safe to ignore; this test must not
  command motion.
- Manual distance markers are placed at `0`, `5`, and `10` meters. Use a tape
  measure, measured floor marks, or a rangefinder. Manual markers are preferred
  over geolocation for short 0/5/10 m tests because laptop/browser location can
  be inaccurate indoors, near buildings, or under trees.

Before walking, verify the relay responds:

```sh
curl --fail http://127.0.0.1:4200/status
```

If `/status` fails at 0 m, stop and fix the relay or Jetson bridge before
running the walk.

## Run

From `couch-bluetooth`:

```sh
nix develop
./couch-range couchvision-walk --relay http://127.0.0.1:4200 --out runs/couchvision-001 --distances 0,5,10 --hold-seconds 30
```

Stand at each marked distance when prompted, press Enter, and hold position
until the station timer finishes. Do not move the couch during this run.

## Analyze And Report

```sh
./couch-range analyze runs/couchvision-001
./couch-range report runs/couchvision-001
```

Open:

- `runs/couchvision-001/report.md`
- `runs/couchvision-001/analysis/distance_summary.csv`
- `runs/couchvision-001/analysis/summary.json`
- `runs/couchvision-001/charts/`
- `runs/couchvision-001/raw/samples.csv`
- `runs/couchvision-001/raw/samples.jsonl`

## Interpreting Output

In this mode, the generic command fields describe the relay heartbeat:

- `command_latency_ms`: CouchVision heartbeat/pong RTT reported by `/status`.
- `command_ack_count`: samples where the relay reports connected and has a
  positive heartbeat RTT.
- `command_timeout_count`: failed `/status` requests, disconnected relay status,
  or missing heartbeat RTT.
- `packet_loss_percent`: heartbeat/status failure rate.
- `reconnect_events` and `drop_events`: relay reachability loss/recovery, not
  proof of BLE reconnects unless the sample explicitly says so.

`reliable` at a distance means the relay reported a connected heartbeat with RTT
inside the configured thresholds at that manual marker. It does not prove
Jetson per-command acknowledgement or safe couch actuation.

Treat a distance as suspect if p95/p99 RTT jumps sharply, any timeout appears,
or the relay reports that the Jetson bridge is disconnected. Fix failures at
0 m before trusting longer-distance results.
