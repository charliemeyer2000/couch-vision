import numpy as np
import pytest

from couch_ekf.ekf import EKF

IDENTITY_QUAT = np.array([0.0, 0.0, 0.0, 1.0])


@pytest.fixture
def ekf_at_origin() -> EKF:
    ekf = EKF()
    ekf.initialize(np.zeros(3), IDENTITY_QUAT)
    return ekf


def test_initialization() -> None:
    ekf = EKF()
    ekf.initialize(np.array([10.0, 20.0, 5.0]), IDENTITY_QUAT)
    assert ekf.initialized
    np.testing.assert_allclose(ekf.x[:3], [10.0, 20.0, 5.0])


def test_not_initialized_by_default() -> None:
    ekf = EKF()
    assert not ekf.initialized


def test_predict_skips_bad_dt(ekf_at_origin: EKF) -> None:
    pos_before = ekf_at_origin.x[:3].copy()
    ekf_at_origin.predict(np.zeros(3), np.zeros(3), IDENTITY_QUAT, -0.1)
    np.testing.assert_array_equal(ekf_at_origin.x[:3], pos_before)
    ekf_at_origin.predict(np.zeros(3), np.zeros(3), IDENTITY_QUAT, 2.0)
    np.testing.assert_array_equal(ekf_at_origin.x[:3], pos_before)

def test_covariance_decreases_after_update(ekf_at_origin: EKF) -> None:
    P_before = ekf_at_origin.P[0, 0]
    R = np.eye(3) * 10.0
    ekf_at_origin.update_gps(np.zeros(3), R)
    assert ekf_at_origin.P[0, 0] < P_before


def test_covariance_stays_symmetric(ekf_at_origin: EKF) -> None:
    accel = np.array([1.0, 0.0, 9.81])
    for _ in range(10):
        ekf_at_origin.predict(accel, np.zeros(3), IDENTITY_QUAT, 0.01)
    ekf_at_origin.update_gps(np.array([1.0, 0.0, 0.0]), np.eye(3) * 5.0)
    np.testing.assert_allclose(ekf_at_origin.P, ekf_at_origin.P.T, atol=1e-10)
