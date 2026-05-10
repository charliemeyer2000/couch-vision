# Subagent Prompts

All subagents were pointed at `/tmp/orin_bluetooth_couch_range_agent_prompt.md`
as the root spec and asked not to edit repo files.

## Device Inventory

Research actual local and tailnet device inventory commands for Linux Bluetooth
adapters, controller input devices, and Jetson/Orin Bluetooth state. Verify
commands available on this machine where possible. Report command list, expected
output shape, failure/degrade behavior, commands that need sudo/root, and
inventory fields the app should collect.

## Measurement Protocol

Design the measurement protocol, reliability thresholds, and raw data schema.
Focus on synthetic command/ack safety, station mode, event markers, CSV/JSONL
columns, summary metrics, and conservative reliability classification. Report
field names and formulas.

## Plotting And Report

Design plotting and Markdown report outputs. Identify necessary charts,
filenames, axes, styling choices, missing-data behavior, and report sections.
Report implementation details without duplicating inventory or schema work.

## Distance Strategy

Investigate distance-estimation options: manual markers, timed stations, event
markers, browser geolocation, phone-assisted GPS/web UI, Wi-Fi/Bluetooth RSSI
approximation, and why each may be inaccurate. Report recommended defaults and
caveats.
