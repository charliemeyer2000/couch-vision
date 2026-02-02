"""Frame sources: abstractions for reading sensor data from bags or live streams.

Provides BagSource which wraps bag_reader with playback pacing, yielding
SyncedFrames at the correct rate for real-time or accelerated replay.
"""

from __future__ import annotations

import time
from collections.abc import Iterator
from dataclasses import dataclass

from couch_perception.bag_reader import (
    GpsFix,
    ImuSample,
    SyncedFrame,
    read_gps_and_imu,
    read_synced_frames,
)


@dataclass
class SensorStreams:
    """All sensor streams from a data source."""

    frames: Iterator[SyncedFrame]
    gps_fixes: list[GpsFix]
    imu_samples: list[ImuSample]


class BagSource:
    """Read frames from an MCAP bag with optional playback pacing.

    Args:
        bag_path: Path to the .mcap bag file.
        playback_rate: Speed multiplier (1.0 = real-time, 0 = as fast as possible).
        max_frames: Stop after this many frames (None = all).
    """

    def __init__(
        self,
        bag_path: str,
        playback_rate: float = 1.0,
        max_frames: int | None = None,
    ) -> None:
        self.bag_path = bag_path
        self.playback_rate = playback_rate
        self.max_frames = max_frames

    def open(self) -> SensorStreams:
        """Open the bag and return all sensor streams."""
        gps_fixes, imu_samples = read_gps_and_imu(self.bag_path)
        frames = self._paced_frames()
        return SensorStreams(
            frames=frames,
            gps_fixes=gps_fixes,
            imu_samples=imu_samples,
        )

    def _paced_frames(self) -> Iterator[SyncedFrame]:
        """Yield frames with playback pacing applied."""
        prev_bag_time: float | None = None
        prev_wall_time: float | None = None
        count = 0

        for frame in read_synced_frames(self.bag_path):
            if self.max_frames is not None and count >= self.max_frames:
                return

            if prev_bag_time is not None and self.playback_rate > 0:
                bag_dt = frame.timestamp - prev_bag_time
                wall_dt = time.monotonic() - prev_wall_time
                sleep_time = (bag_dt / self.playback_rate) - wall_dt
                if sleep_time > 0:
                    time.sleep(sleep_time)

            prev_bag_time = frame.timestamp
            prev_wall_time = time.monotonic()
            count += 1
            yield frame
