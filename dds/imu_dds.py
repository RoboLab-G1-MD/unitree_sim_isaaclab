# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""ImuDDS — publishes rt/dog_imu_raw (sensor_msgs/Imu) from Isaac Lab imu_in_torso body.

Publishes standard ROS2 sensor_msgs/Imu on topic rt/dog_imu_raw (= ROS2 /dog_imu_raw).
Frame id: imu_in_torso.

Light Gaussian noise is added to accelerometer and gyroscope to simulate
a real MEMS IMU (no bias drift, white noise only):
  accel:  σ = 0.02 m/s²
  gyro:   σ = 0.005 rad/s
"""

import time
import numpy as np
from dataclasses import dataclass, field
from typing import List

import dds.sim_time as sim_time

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types

from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.std_msgs.msg.dds_ import Header_
from unitree_sdk2py.idl.builtin_interfaces.msg.dds_ import Time_

from dds.dds_base import DDSObject

_TOPIC   = "rt/dog_imu_raw"
_FRAME_ID = "dog_imu_link"

# Noise standard deviations
_ACCEL_NOISE_STD = 0.02    # m/s²
_GYRO_NOISE_STD  = 0.005   # rad/s


# ---------------------------------------------------------------------------
# IDL type definitions  (sensor_msgs/Imu wire format)
# ---------------------------------------------------------------------------

@dataclass
@annotate.final
@annotate.autoid("sequential")
class _Quaternion(idl.IdlStruct, typename="geometry_msgs.msg.dds_.Quaternion_"):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
@annotate.final
@annotate.autoid("sequential")
class _Vector3(idl.IdlStruct, typename="geometry_msgs.msg.dds_.Vector3_"):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
@annotate.final
@annotate.autoid("sequential")
class Imu_(idl.IdlStruct, typename="sensor_msgs.msg.dds_.Imu_"):
    header:                        Header_
    orientation:                   _Quaternion
    orientation_covariance:        types.array[float, 9]
    angular_velocity:              _Vector3
    angular_velocity_covariance:   types.array[float, 9]
    linear_acceleration:           _Vector3
    linear_acceleration_covariance: types.array[float, 9]


# ---------------------------------------------------------------------------
# Helper: rotate world-frame vector to body frame given quaternion (w,x,y,z)
# ---------------------------------------------------------------------------

def _quat_to_rot_w2b(q_wxyz: np.ndarray) -> np.ndarray:
    """Return 3×3 R_world→body  (= R_body→world^T)."""
    w, x, y, z = q_wxyz
    R_b2w = np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - w*z),   2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),  2*(y*z - w*x)],
        [    2*(x*z - w*y),   2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)
    return R_b2w.T


_prev_vel   = None
_prev_time  = None


class ImuDDS(DDSObject):
    """Publishes sensor_msgs/Imu to DDS topic rt/dog_imu_raw."""

    node_name = "ImuDDS"

    def __init__(self, env):
        super().__init__()
        self._env = env
        self._publisher = None
        self._msg = None

    def setup_publisher(self):
        self._publisher = ChannelPublisher(_TOPIC, Imu_)
        self._publisher.Init()

        zero9 = [0.0] * 9
        self._msg = Imu_(
            header=Header_(stamp=Time_(sec=0, nanosec=0), frame_id=_FRAME_ID),
            orientation=_Quaternion(),
            orientation_covariance=zero9,
            angular_velocity=_Vector3(),
            angular_velocity_covariance=zero9,
            linear_acceleration=_Vector3(),
            linear_acceleration_covariance=zero9,
        )
        print(f"[{self.node_name}] Publisher ready on {_TOPIC}")

    def setup_subscriber(self):
        pass

    def dds_subscriber(self, msg, datatype=None):
        pass

    def dds_publisher(self):
        global _prev_vel, _prev_time

        try:
            data = self._env.scene["robot"].data

            # Use root_state_w — thread-safe (pre-cached tensor, no PhysX getGlobalPose call).
            # body_link_pose_w triggers getGlobalPose() during simulation and causes PhysX errors.
            root = data.root_state_w[0].cpu().numpy()
            pos           = root[:3]
            quat          = root[3:7]    # w, x, y, z  (Isaac Lab w-first)
            lin_vel       = root[7:10]
            ang_vel_world = root[10:13]

            # --- angular velocity in body frame ---
            R_w2b = _quat_to_rot_w2b(quat)
            ang_vel_body = R_w2b @ ang_vel_world

            # --- linear acceleration in body frame (numerical diff) ---
            now = time.monotonic()
            if _prev_vel is None or _prev_time is None:
                dt = 0.01
            else:
                dt = max(now - _prev_time, 1e-6)
            a_world = (lin_vel - _prev_vel) / dt if _prev_vel is not None else np.zeros(3)
            _prev_vel  = lin_vel.copy()
            _prev_time = now

            g_world = np.array([0.0, 0.0, -9.81])
            acc_body = R_w2b @ (a_world - g_world)

            # --- add light Gaussian noise ---
            acc_body  = acc_body  + np.random.normal(0.0, _ACCEL_NOISE_STD, 3)
            ang_vel_body = ang_vel_body + np.random.normal(0.0, _GYRO_NOISE_STD, 3)

            # --- fill message ---
            stamp = sim_time.now_stamp()
            msg = self._msg
            msg.header.stamp.sec    = stamp.sec
            msg.header.stamp.nanosec = stamp.nanosec

            # Isaac Lab quaternion: w-first [w, x, y, z]
            msg.orientation.w = float(quat[0])
            msg.orientation.x = float(quat[1])
            msg.orientation.y = float(quat[2])
            msg.orientation.z = float(quat[3])

            msg.angular_velocity.x = float(ang_vel_body[0])
            msg.angular_velocity.y = float(ang_vel_body[1])
            msg.angular_velocity.z = float(ang_vel_body[2])

            msg.linear_acceleration.x = float(acc_body[0])
            msg.linear_acceleration.y = float(acc_body[1])
            msg.linear_acceleration.z = float(acc_body[2])

            self._publisher.Write(msg)

        except Exception as e:
            print(f"[{self.node_name}] publish error: {e}")
