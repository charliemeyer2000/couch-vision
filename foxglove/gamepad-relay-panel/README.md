# Gamepad Relay Panel

Foxglove panel that shows live gamepad state from the native SDL relay (`scripts/gamepad_relay.py`).

Polls `http://localhost:4201/state` at 20Hz and displays stick positions, button states, cmd_vel output, and BLE connection status.

Requires `make teleop` (or `make gamepad-relay`) running on the Mac.
