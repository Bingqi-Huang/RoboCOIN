#!/usr/bin/env python

"""Record a short RealSense color video into the repo's results/ folder."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import time

import cv2
import numpy as np
import pyrealsense2 as rs


CAMERA_SERIAL = "317222074160"
WIDTH = 640
HEIGHT = 480
FPS = 15
RECORD_SECONDS = 100000

ROOT_DIR = Path(__file__).resolve().parents[1]
RESULTS_DIR = ROOT_DIR / "results"


def main() -> None:
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    output_path = RESULTS_DIR / (
        f"realsense_{WIDTH}x{HEIGHT}_{FPS}fps_{datetime.now():%Y%m%d_%H%M%S}.mp4"
    )

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(CAMERA_SERIAL)
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)

    try:
        pipeline.start(config)
    except RuntimeError as exc:
        raise SystemExit(f"Could not start RealSense camera {CAMERA_SERIAL}: {exc}") from exc

    writer = cv2.VideoWriter(
        str(output_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        FPS,
        (WIDTH, HEIGHT),
    )
    if not writer.isOpened():
        pipeline.stop()
        raise SystemExit(f"Could not open video writer for {output_path}")

    print(f"Recording {RECORD_SECONDS} seconds to {output_path}")

    try:
        time.sleep(1.0)
        start_time = time.time()
        while time.time() - start_time < RECORD_SECONDS:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())
            if frame.shape[:2] != (HEIGHT, WIDTH):
                frame = cv2.resize(frame, (WIDTH, HEIGHT))

            writer.write(frame)
    except KeyboardInterrupt:
        print("Stopped early with Ctrl+C.")
    finally:
        writer.release()
        pipeline.stop()

    print(f"Saved video to {output_path}")


if __name__ == "__main__":
    main()
