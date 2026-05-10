import pytest
import mujoco

from couch_sim.config import available_scenarios, load_scenario, load_vehicle
from couch_sim.model import build_mjcf


def test_generated_models_compile_for_all_scenarios() -> None:
    vehicle = load_vehicle()
    for scenario_name in available_scenarios():
        scenario = load_scenario(scenario_name)
        xml = build_mjcf(vehicle, scenario)
        model = mujoco.MjModel.from_xml_string(xml)
        assert model.nbody > 0


def test_model_has_compliant_front_frame_joints() -> None:
    vehicle = load_vehicle()
    scenario = load_scenario("flat_default")
    model = mujoco.MjModel.from_xml_string(build_mjcf(vehicle, scenario))
    pitch_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "frame_flex_pitch")
    roll_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "frame_flex_roll")
    assert pitch_joint >= 0
    assert roll_joint >= 0


def test_front_caster_contact_is_soft_tire_envelope() -> None:
    vehicle = load_vehicle()
    scenario = load_scenario("flat_default")
    model = mujoco.MjModel.from_xml_string(build_mjcf(vehicle, scenario))
    caster_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "front_left_caster_collision")

    assert model.geom_size[caster_geom][0] > vehicle.caster_radius
    assert 0.010 <= model.geom_margin[caster_geom] <= 0.014
    assert 0.012 <= model.geom_solref[caster_geom][0] <= 0.016
    assert 0.65 <= model.geom_solimp[caster_geom][0] <= 0.70


def test_rider_center_of_gravity_is_lowered_eight_inches() -> None:
    vehicle = load_vehicle()
    scenario = load_scenario("flat_default")
    model = mujoco.MjModel.from_xml_string(build_mjcf(vehicle, scenario))
    rider = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "central_rider_mass")

    assert vehicle.com_height == pytest.approx(0.4168)
    assert model.body_pos[rider][2] < 0.30


def test_intersecting_grades_model_has_two_grade_geoms() -> None:
    vehicle = load_vehicle()
    scenario = load_scenario("intersecting_grades")
    model = mujoco.MjModel.from_xml_string(build_mjcf(vehicle, scenario))
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "grade_north_south") >= 0
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "grade_east_west") >= 0


def test_rider_mass_keeps_yaw_inertia_centered() -> None:
    vehicle = load_vehicle()
    scenario = load_scenario("flat_default")
    model = mujoco.MjModel.from_xml_string(build_mjcf(vehicle, scenario))
    chassis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    rider = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "central_rider_mass")
    assert model.body_mass[chassis] == vehicle.frame_mass
    assert model.body_mass[rider] == vehicle.rider_mass
    assert model.body_inertia[rider][2] < 8.0
