# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""LiDAR DDS publisher — publishes rt/utlidar/cloud_livox_mid360 as sensor_msgs/PointCloud2."""

import struct
import time
import numpy as np
from multiprocessing import shared_memory

from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointField_
from unitree_sdk2py.idl.std_msgs.msg.dds_ import Header_
from unitree_sdk2py.idl.builtin_interfaces.msg.dds_ import Time_

from dds.dds_base import DDSObject

_LIDAR_SHM_NAME = "isaac_lidar_shm"
_LIDAR_TOPIC = "rt/utlidar/cloud_livox_mid360"
_FRAME_ID = "mid360_link"

# PointField datatype constants (matches sensor_msgs/PointField)
_FLOAT32 = 7


class LidarDDS(DDSObject):
    """Publishes LiDAR point cloud to DDS topic rt/utlidar/cloud_livox_mid360."""

    node_name = "LidarDDS"

    def __init__(self):
        super().__init__()
        self._shm = None
        self._buf = None
        self._publisher = None
        self._msg = self._build_empty_msg()

    # ------------------------------------------------------------------
    # DDSObject interface
    # ------------------------------------------------------------------

    def setup_publisher(self):
        self._publisher = ChannelPublisher(_LIDAR_TOPIC, PointCloud2_)
        self._publisher.Init()
        print(f"[{self.node_name}] Publisher ready on {_LIDAR_TOPIC}")

    def setup_subscriber(self):
        pass  # publish-only

    def dds_subscriber(self, msg, datatype=None):
        pass

    def dds_publisher(self):
        """Read XYZ from shared memory and publish PointCloud2."""
        xyz = self._read_shm()
        if xyz is None:
            return

        num_points = xyz.shape[0]
        raw = xyz.tobytes()

        now = time.time()
        sec = int(now)
        nanosec = int((now - sec) * 1e9)

        self._msg.header.stamp.sec = sec
        self._msg.header.stamp.nanosec = nanosec
        self._msg.width = num_points
        self._msg.row_step = num_points * 12
        self._msg.data = list(raw)

        self._publisher.Write(self._msg)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _read_shm(self):
        """Return float32 array (N,3) or None if shm not ready."""
        try:
            if self._shm is None:
                self._shm = shared_memory.SharedMemory(name=_LIDAR_SHM_NAME)
                self._buf = np.ndarray((self._shm.size,), dtype=np.uint8, buffer=self._shm.buf)
        except FileNotFoundError:
            return None

        num_points = struct.unpack_from("<I", self._buf[:4])[0]
        if num_points == 0:
            return None

        expected = 4 + num_points * 12
        if self._shm.size < expected:
            return None

        xyz = np.frombuffer(bytes(self._buf[4:expected]), dtype=np.float32).reshape(num_points, 3).copy()
        return xyz

    @staticmethod
    def _build_empty_msg() -> PointCloud2_:
        fields = [
            PointField_(name="x", offset=0,  datatype=_FLOAT32, count=1),
            PointField_(name="y", offset=4,  datatype=_FLOAT32, count=1),
            PointField_(name="z", offset=8,  datatype=_FLOAT32, count=1),
        ]
        msg = PointCloud2_(
            header=Header_(stamp=Time_(sec=0, nanosec=0), frame_id=_FRAME_ID),
            height=1,
            width=0,
            fields=fields,
            is_bigendian=False,
            point_step=12,
            row_step=0,
            data=[],
            is_dense=True,
        )
        return msg
