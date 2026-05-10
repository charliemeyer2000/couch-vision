from pathlib import Path

from couch_sim.config import available_scenarios, load_scenario, load_vehicle, simulator_root


def test_load_default_vehicle() -> None:
    vehicle = load_vehicle()
    assert vehicle.name == "couch_default"
    assert vehicle.mass_total > 0
    assert vehicle.wheel_separation > 0


def test_all_named_scenarios_load() -> None:
    for name in available_scenarios():
        scenario = load_scenario(name)
        assert scenario.name == name
        assert scenario.surface_length > 0


def test_intersecting_grades_has_two_crossing_planes() -> None:
    scenario = load_scenario("intersecting_grades")
    assert scenario.terrain_planes is not None
    assert len(scenario.terrain_planes) == 2
    assert {plane.yaw_deg for plane in scenario.terrain_planes} == {0.0, 90.0}
    assert all(plane.slope_pitch_deg == 5.0 for plane in scenario.terrain_planes)


def test_explicit_vehicle_override_path() -> None:
    vehicle_path = simulator_root() / "config/vehicle/default.yaml"
    vehicle = load_vehicle(Path(vehicle_path))
    assert vehicle.camera_distance > 0
