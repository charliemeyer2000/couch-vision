from __future__ import annotations

import math

# WGS84 ellipsoid parameters
_A = 6378137.0  # semi-major axis (m)
_F = 1.0 / 298.257223563
_B = _A * (1.0 - _F)
_E2 = 1.0 - (_B * _B) / (_A * _A)

# Reference point: UVA Rotunda
ROTUNDA_LAT = 38.035853
ROTUNDA_LON = -78.503307
ROTUNDA_ALT = 174.0  # approximate elevation (m)


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt: float) -> tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)
    n = _A / math.sqrt(1.0 - _E2 * sin_lat * sin_lat)
    x = (n + alt) * cos_lat * cos_lon
    y = (n + alt) * cos_lat * sin_lon
    z = (n * (1.0 - _E2) + alt) * sin_lat
    return x, y, z


def ecef_to_enu(
    x: float, y: float, z: float,
    ref_lat_deg: float, ref_lon_deg: float, ref_alt: float,
) -> tuple[float, float, float]:
    xr, yr, zr = geodetic_to_ecef(ref_lat_deg, ref_lon_deg, ref_alt)
    dx, dy, dz = x - xr, y - yr, z - zr
    lat = math.radians(ref_lat_deg)
    lon = math.radians(ref_lon_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)
    e = -sin_lon * dx + cos_lon * dy
    n = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    u = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return e, n, u


def geodetic_to_enu(
    lat_deg: float, lon_deg: float, alt: float,
    ref_lat: float = ROTUNDA_LAT,
    ref_lon: float = ROTUNDA_LON,
    ref_alt: float = ROTUNDA_ALT,
) -> tuple[float, float, float]:
    x, y, z = geodetic_to_ecef(lat_deg, lon_deg, alt)
    return ecef_to_enu(x, y, z, ref_lat, ref_lon, ref_alt)
