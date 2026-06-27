from __future__ import annotations

import math

from couch_perception.geo import geodetic_to_ecef, geodetic_to_enu


def test_geodetic_to_ecef_at_equator_prime_meridian() -> None:
    """A point on the equator at 0 lon should lie on the +X axis."""
    x, y, z = geodetic_to_ecef(0.0, 0.0, 0.0)
    assert x > 6_000_000
    assert abs(y) < 1.0
    assert abs(z) < 1.0


def test_geodetic_to_enu_identity_at_reference() -> None:
    """Converting the reference point itself should give ~(0, 0, 0)."""
    from couch_perception.geo import ROTUNDA_LAT, ROTUNDA_LON, ROTUNDA_ALT

    e, n, u = geodetic_to_enu(ROTUNDA_LAT, ROTUNDA_LON, ROTUNDA_ALT)
    assert math.isclose(e, 0.0, abs_tol=1e-6)
    assert math.isclose(n, 0.0, abs_tol=1e-6)
    assert math.isclose(u, 0.0, abs_tol=1e-6)
