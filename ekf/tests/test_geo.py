import pytest

from couch_ekf.geo import ROTUNDA_ALT, ROTUNDA_LAT, ROTUNDA_LON, geodetic_to_enu


def test_origin_is_zero() -> None:
    e, n, u = geodetic_to_enu(ROTUNDA_LAT, ROTUNDA_LON, ROTUNDA_ALT)
    assert e == pytest.approx(0.0, abs=1e-6)
    assert n == pytest.approx(0.0, abs=1e-6)
    assert u == pytest.approx(0.0, abs=1e-6)


def test_north_displacement() -> None:
    e, n, u = geodetic_to_enu(ROTUNDA_LAT + 0.001, ROTUNDA_LON, ROTUNDA_ALT)
    assert e == pytest.approx(0.0, abs=1.0)
    assert 100 < n < 120
    assert u == pytest.approx(0.0, abs=1.0)


def test_east_displacement() -> None:
    e, n, u = geodetic_to_enu(ROTUNDA_LAT, ROTUNDA_LON + 0.001, ROTUNDA_ALT)
    assert 70 < e < 100
    assert n == pytest.approx(0.0, abs=1.0)
    assert u == pytest.approx(0.0, abs=1.0)


def test_altitude() -> None:
    e, n, u = geodetic_to_enu(ROTUNDA_LAT, ROTUNDA_LON, ROTUNDA_ALT + 50.0)
    assert e == pytest.approx(0.0, abs=1.0)
    assert n == pytest.approx(0.0, abs=1.0)
    assert u == pytest.approx(50.0, abs=1.0)


def test_roundtrip_symmetry() -> None:
    """Moving east then west should cancel out."""
    e1, n1, _ = geodetic_to_enu(ROTUNDA_LAT, ROTUNDA_LON + 0.01, ROTUNDA_ALT)
    e2, n2, _ = geodetic_to_enu(ROTUNDA_LAT, ROTUNDA_LON - 0.01, ROTUNDA_ALT)
    assert e1 == pytest.approx(-e2, rel=1e-3)
    assert n1 == pytest.approx(n2, abs=1.0)
