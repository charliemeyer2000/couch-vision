# Couch Simulator

Standalone MuJoCo sandbox for tuning manual couch teleop feel.

## Run

```bash
cd sim
uv sync
uv run couch-sim
```

## Useful commands

```bash
uv run couch-sim --headless --steps 600 --log-path output/logs/smoke.json
uv run couch-sim-check --bag ../bags/2026-05-08_18-25-49_0.mcap --max-duration 12
uv run couch-sim --scenario cross_slope --mode assisted
uv run couch-sim-plot output/logs/raw.json --compare output/logs/assisted.json
```

`couch-sim-check` is the headless regression loop for drivetrain tuning. It replays a real
turn window from the MCAP, compares the simulated yaw-rate envelope against `/wheel_odom`,
and fails if chassis hop or wheel/caster airborne streaks exceed the acceptance limits.

## Controls

- `W/A/S/D`: drive
- `Space`: zero command
- `Tab`: raw vs assisted
- `1-6`: scenario presets
- `R`: reset current scenario
- `,` / `.`: change active tuning parameter
- `[` / `]`: decrease/increase active tuning value
- Mouse drag in the on-screen joystick: analog drive input
- Mouse drag in the scene: orbit camera
- Mouse wheel: zoom camera
