from couch_range.distance import build_stations, parse_distances


def test_parse_distances() -> None:
    assert parse_distances("0, 5,10", [1]) == [0.0, 5.0, 10.0]


def test_parse_distances_uses_default() -> None:
    assert parse_distances("", [1, 2]) == [1, 2]


def test_build_stations() -> None:
    stations = build_stations([0, 5], 30, "manual_marker")
    assert [station.meters for station in stations] == [0, 5]
    assert stations[0].hold_seconds == 30
