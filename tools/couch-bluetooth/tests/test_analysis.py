from couch_range.analysis import classify, percentile
from couch_range.config import Thresholds
from couch_range.telemetry import ReliabilityBand


def test_percentile_interpolates() -> None:
    assert percentile([10, 20, 30], 50) == 20
    assert percentile([10, 20, 30], 95) == 29


def test_classify_reliable() -> None:
    band = classify(0.995, 80, 0.5, 0, 0, Thresholds())
    assert band == ReliabilityBand.RELIABLE


def test_classify_unsafe_on_latency() -> None:
    band = classify(0.995, 500, 0.5, 0, 0, Thresholds())
    assert band == ReliabilityBand.UNSAFE
