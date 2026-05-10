from couch_sim.app import RuntimeState, _build_bundle, _start_camera_orbit, _update_camera_orbit, _zoom_camera
from couch_sim.config import load_scenario, load_vehicle


def test_scene_mouse_drag_orbits_camera_without_drive_input() -> None:
    vehicle = load_vehicle()
    scenario = load_scenario("flat_default")
    bundle = _build_bundle(vehicle, scenario, enable_rendering=True)
    runtime = RuntimeState(scenario_name="flat_default", mode="raw", vehicle=vehicle)

    assert bundle.camera is not None
    start_azimuth = float(bundle.camera.azimuth)
    start_elevation = float(bundle.camera.elevation)

    _start_camera_orbit(runtime, (100, 100))
    _update_camera_orbit(bundle, runtime, (140, 120))

    assert runtime.analog_forward == 0.0
    assert runtime.analog_turn == 0.0
    assert bundle.camera.azimuth > start_azimuth
    assert bundle.camera.elevation > start_elevation


def test_mouse_wheel_zooms_camera() -> None:
    vehicle = load_vehicle()
    scenario = load_scenario("flat_default")
    bundle = _build_bundle(vehicle, scenario, enable_rendering=True)

    assert bundle.camera is not None
    start_distance = float(bundle.camera.distance)
    _zoom_camera(bundle, 1)

    assert bundle.camera.distance < start_distance
