## 0.2.0

- Replace per-wheel ERPM ramp UI (`Ramp ↑/↓ RPM/s`) with cmd_vel slew in
  intuitive units: separate Linear ↑/↓ (m/s²) and Angular ↑/↓ (rad/s²) inputs.
  Steering can now be tuned much faster than throttle, eliminating the
  hill-contour turning lag where the inside wheel couldn't decelerate fast
  enough to allow a sharp turn at speed.
- Add **Bypass slew** toggle for raw cmd_vel passthrough during tuning.
- Add **Slew lag** monitor (raw vs commanded linear/angular bars) so the
  controller lag is visible at a glance.
- Add **Wheel telemetry** section: per-wheel actual vs commanded RPM bars
  plus per-wheel lag readouts. Subscribes to new Float64 topics
  `/motor/wheel_{left,right}/{rpm,cmd_rpm}` and `/motor/cmd_vel/{linear,angular}_{raw,slewed}`.
- Promote the cmd_vel transport indicator to a **prominent banner** right
  below E-Stop: green "BLE FAST PATH" when active, amber "Wi-Fi fallback"
  when BLE relay is unreachable, gray "Wi-Fi (WebSocket)" when BLE is
  toggled off. Also adds a small "via BLE / via Wi-Fi" pill next to the
  Velocity Monitor header so the path is visible at any scroll position.
  Removes the old buried BLE Fast Path toggle in Motor Config (now in the banner).

## 0.1.0

- Initial release: E-stop, teleop joystick, velocity monitor, system status
