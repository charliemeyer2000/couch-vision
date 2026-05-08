# CouchVision Integration Map

`couch-bluetooth` should stay the field-test and reporting repo. It owns the
CLI, Nix shell, telemetry schema, station runner, inventory, analysis, plotting,
and Markdown report flow.

`couch-vision` should remain the couch-control/runtime repo. It owns ROS2,
perception, Foxglove panels, BLE-to-ROS bridging, VESC serial control, `/cmd_vel`,
`/e_stop`, and `/motor/config`.

## Current CouchVision Control Path

Observed path from `/home/barrett/dev/couch-vision`:

```text
Foxglove panel
  -> localhost:4200 HTTP
  -> scripts/ble_relay.py
  -> BLE notify
  -> perception/src/couch_perception/ble_bridge.py
  -> ROS2 /cmd_vel or /e_stop
  -> perception/src/couch_perception/vesc_driver.py
  -> VESC serial
  -> motors
```

Key files:

- `/home/barrett/dev/couch-vision/scripts/ble_relay.py`
- `/home/barrett/dev/couch-vision/perception/src/couch_perception/ble_bridge.py`
- `/home/barrett/dev/couch-vision/perception/src/couch_perception/vesc_driver.py`
- `/home/barrett/dev/couch-vision/foxglove/hardware-safety-panel/src/HardwareSafetyPanel.tsx`
- `/home/barrett/dev/couch-vision/perception/docker-compose.vesc.yml`
- `/home/barrett/dev/couch-vision/perception/entrypoint_ble.sh`

## Missing Test Surface

The current CouchVision BLE relay does not acknowledge each couch command from
the Jetson side. A local `POST /cmd_vel` success only proves the relay accepted a
request and updated a BLE characteristic. It does not prove the Jetson received
that command.

The range tester needs a no-motion command/ack surface:

- `command_kind = "link_check"` or `"controller_sample"`
- sequence-numbered
- `requires_actuation = false`
- acknowledged by the Jetson side
- never published to `/cmd_vel`

## Where Code Belongs

Put in `couch-bluetooth`:

- transport abstraction
- current TCP synthetic transport
- BLE test runner
- GATT UUID constants compatible with CouchVision
- heartbeat/pong status polling
- per-command telemetry
- controller sample logging
- plots and reports

Keep in `couch-vision`:

- ROS2 graph
- Foxglove UI
- BLE-to-ROS bridge
- VESC driver
- actuator commands
- Docker/Jetson deployment

Wrap the CouchVision BLE shape. Do not vendor `rclpy`, Nav2, perception, or VESC
runtime code into this test repo.

## First Patch Sequence

1. Add a `Transport` interface in `couch-bluetooth`; keep TCP as the baseline.
2. Add optional BLE dependencies and runtime probes: `bleak`, `bless`, `aiohttp`,
   `btmgmt`, `bluetoothctl`.
3. Add `couch-range ble-run` for laptop peripheral mode and
   `couch-range ble-receiver` for Jetson central mode.
4. Log BLE connected state, RTT, ack latency, drops, reconnects, and errors into
   existing sample fields.
5. Add the matching no-motion ack endpoint/characteristic in `couch-vision` only
   when testing against the live relay/bridge process.
6. After synthetic BLE is reliable, add wired-controller sample commands with
   `requires_actuation = false`.
7. Actual `/cmd_vel` testing comes last.

## Safety Gates

- No couch motion in ping-only, synthetic TCP, or synthetic BLE phases.
- Do not treat zero-velocity `/cmd_vel` as proof of safe receipt.
- Avoid port `8765` conflicts with CouchVision/Foxglove/Nav2 when both stacks run.
- Current CouchVision BLE direction is laptop/Mac peripheral and Jetson central;
  do not invert that unless the Jetson advertising mode is verified.
- Before actuation: physical e-stop present, VESC e-stopped, conservative max
  RPM, synthetic BLE ack reliable, no repeated disconnects, no missed acks.
