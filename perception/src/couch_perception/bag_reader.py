"""Read compressed images from MCAP bag files."""

from collections.abc import Iterator
from pathlib import Path

import cv2
import numpy as np


def read_compressed_images(bag_path: str | Path, topic_suffix: str = "image/compressed") -> Iterator[tuple[float, np.ndarray]]:
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    with Path(bag_path).open("rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for _schema, channel, message, ros_msg in reader.iter_decoded_messages():
            if not channel.topic.endswith(topic_suffix):
                continue
            if not (hasattr(ros_msg, "format") and hasattr(ros_msg, "data")):
                continue
            buf = np.frombuffer(ros_msg.data, dtype=np.uint8)
            img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if img is not None:
                yield message.log_time / 1e9, img
