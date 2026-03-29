# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache-2.0
"""Sensor publisher: DDS LiDAR point cloud + DDS RGB/Depth camera images."""

import struct
import threading
import time
import numpy as np
from multiprocessing import shared_memory

from tools.shared_memory_utils import MultiImageReader

_DEPTH_SHM_NAME = "isaac_depth_shm"


class SensorPublisher:
    """Runs DDS LiDAR and DDS camera publishers in daemon threads."""

    def __init__(self, enable_lidar: bool = True,
                 enable_rgb: bool = True,
                 enable_depth: bool = True,
                 lidar_hz: float = 10.0,
                 image_hz: float = 15.0):
        self._enable_lidar = enable_lidar
        self._enable_rgb   = enable_rgb
        self._enable_depth = enable_depth
        self._lidar_hz  = lidar_hz
        self._image_hz  = image_hz
        self._running   = False
        self._threads   = []

        self._image_reader = MultiImageReader() if enable_rgb else None
        self._depth_shm    = None
        self._depth_buf    = None

        self._lidar_dds  = None
        self._camera_dds = None

        if enable_lidar:
            from dds.lidar_dds import LidarDDS
            self._lidar_dds = LidarDDS()
            self._lidar_dds.setup_publisher()

        if enable_rgb or enable_depth:
            from dds.camera_dds import CameraDDS
            self._camera_dds = CameraDDS()
            self._camera_dds.setup_publishers()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self):
        self._running = True

        if self._enable_lidar and self._lidar_dds is not None:
            self._threads.append(self._spawn(self._lidar_loop))

        if (self._enable_rgb or self._enable_depth) and self._camera_dds is not None:
            self._threads.append(self._spawn(self._camera_loop))

        print("[SensorPublisher] Started")

    def stop(self):
        self._running = False
        for t in self._threads:
            t.join(timeout=1.0)

    # ------------------------------------------------------------------
    # Publisher loops
    # ------------------------------------------------------------------

    def _lidar_loop(self):
        interval = 1.0 / self._lidar_hz
        while self._running:
            t0 = time.time()
            try:
                self._lidar_dds.dds_publisher()
            except Exception as e:
                print(f"[SensorPublisher] lidar error: {e}")
            time.sleep(max(0.0, interval - (time.time() - t0)))

    def _camera_loop(self):
        """Single loop for both RGB and depth at image_hz."""
        interval = 1.0 / self._image_hz
        while self._running:
            t0 = time.time()

            if self._enable_rgb:
                try:
                    images = self._image_reader.read_images()
                    if images and "head" in images:
                        self._camera_dds.publish_color(images["head"])
                except Exception as e:
                    print(f"[SensorPublisher] RGB error: {e}")

            if self._enable_depth:
                try:
                    depth = self._read_depth_shm()
                    if depth is not None:
                        self._camera_dds.publish_depth(depth)
                except Exception as e:
                    print(f"[SensorPublisher] depth error: {e}")

            time.sleep(max(0.0, interval - (time.time() - t0)))

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _read_depth_shm(self) -> np.ndarray | None:
        """Return float32 (H, W) array or None."""
        try:
            if self._depth_shm is None:
                self._depth_shm = shared_memory.SharedMemory(name=_DEPTH_SHM_NAME)
                self._depth_buf = np.ndarray(
                    (self._depth_shm.size,), dtype=np.uint8, buffer=self._depth_shm.buf)
        except FileNotFoundError:
            return None

        h, w = struct.unpack_from("<II", self._depth_buf[:8])
        if h == 0 or w == 0:
            return None
        return np.frombuffer(
            bytes(self._depth_buf[8:8 + h * w * 4]), dtype=np.float32
        ).reshape(h, w).copy()

    @staticmethod
    def _spawn(target) -> threading.Thread:
        t = threading.Thread(target=target, daemon=True)
        t.start()
        return t


def start_sensor_publisher(**kwargs) -> SensorPublisher:
    pub = SensorPublisher(**kwargs)
    pub.start()
    return pub
