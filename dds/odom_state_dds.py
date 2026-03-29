# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""OdomStateDDS — publishes rt/odommodestate (SportModeState_) from Isaac Lab ground-truth.

Fills the minimal fields read by OdomBridge in g1_state_bridge:
  position[3]           – XYZ in world frame (metres)
  velocity[3]           – linear velocity in world frame (m/s)
  yaw_speed             – angular velocity around Z (rad/s)
  imu_state.quaternion  – [w, x, y, z]
  imu_state.gyroscope   – angular velocity in body frame (rad/s)
  imu_state.accelerometer – specific force in body frame (m/s²)
"""

import time
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_

from dds.dds_base import DDSObject

_TOPIC = "rt/odommodestate"


class OdomStateDDS(DDSObject):
    """Publishes ground-truth odometry from Isaac Lab as SportModeState_ on rt/odommodestate."""

    node_name = "OdomStateDDS"

    def __init__(self, env):
        super().__init__()
        self._env = env
        self._publisher = None
        self._msg = unitree_go_msg_dds__SportModeState_()

    # ------------------------------------------------------------------
    # DDSObject interface
    # ------------------------------------------------------------------

    def setup_publisher(self):
        self._publisher = ChannelPublisher(_TOPIC, SportModeState_)
        self._publisher.Init()
        print(f"[{self.node_name}] Publisher ready on {_TOPIC}")

    def setup_subscriber(self):
        pass  # publish-only

    def dds_subscriber(self, msg, datatype=None):
        pass

    def dds_publisher(self):
        """Read ground-truth pose/velocity from Isaac Lab and publish SportModeState_."""
        try:
            data = self._env.scene["robot"].data

            # --- position (world frame) ---
            pos = data.root_pos_w[0].cpu().numpy()          # (3,) float32
            # --- linear velocity (world frame) ---
            lin_vel = data.root_lin_vel_w[0].cpu().numpy()  # (3,)
            # --- angular velocity (world frame) ---
            ang_vel_w = data.root_ang_vel_w[0].cpu().numpy()  # (3,)
            # --- quaternion (w, x, y, z) — Isaac Lab uses w-first ---
            quat = data.root_quat_w[0].cpu().numpy()         # (4,) [w, x, y, z]

            # rotate angular velocity to body frame: omega_body = R_w2b @ omega_world
            ang_vel_b = _rotate_to_body(quat, ang_vel_w)

            # approximate specific force in body frame (gravity-subtracted)
            acc_b = _compute_acc_body(self, quat, lin_vel)

            # --- fill message ---
            msg = self._msg
            msg.position[0] = float(pos[0])
            msg.position[1] = float(pos[1])
            msg.position[2] = float(pos[2])

            msg.velocity[0] = float(lin_vel[0])
            msg.velocity[1] = float(lin_vel[1])
            msg.velocity[2] = float(lin_vel[2])

            msg.yaw_speed = float(ang_vel_w[2])

            imu = msg.imu_state
            imu.quaternion[0] = float(quat[0])  # w
            imu.quaternion[1] = float(quat[1])  # x
            imu.quaternion[2] = float(quat[2])  # y
            imu.quaternion[3] = float(quat[3])  # z

            imu.gyroscope[0] = float(ang_vel_b[0])
            imu.gyroscope[1] = float(ang_vel_b[1])
            imu.gyroscope[2] = float(ang_vel_b[2])

            imu.accelerometer[0] = float(acc_b[0])
            imu.accelerometer[1] = float(acc_b[1])
            imu.accelerometer[2] = float(acc_b[2])

            self._publisher.Write(msg)
        except Exception as e:
            print(f"[{self.node_name}] publish error: {e}")


# ------------------------------------------------------------------
# Helpers (module-level to avoid instance overhead)
# ------------------------------------------------------------------

def _quat_to_rot_world_to_body(q_wxyz: np.ndarray) -> np.ndarray:
    """Return 3×3 rotation matrix R such that v_body = R @ v_world."""
    w, x, y, z = q_wxyz
    R = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y + w*z),     2*(x*z - w*y)],
        [    2*(x*y - w*z), 1 - 2*(x*x + z*z),     2*(y*z + w*x)],
        [    2*(x*z + w*y),     2*(y*z - w*x), 1 - 2*(x*x + y*y)],
    ], dtype=np.float32)
    return R  # R is R_body_to_world; transpose for world_to_body
    # actually: v_body = R^T @ v_world


def _rotate_to_body(q_wxyz: np.ndarray, v_world: np.ndarray) -> np.ndarray:
    R_b2w = _quat_to_rot_world_to_body(q_wxyz)  # body→world
    return R_b2w.T @ v_world                      # world→body


# Per-instance velocity cache stored as module-level dict keyed by object id
_vel_cache: dict = {}

def _compute_acc_body(obj, q_wxyz: np.ndarray, lin_vel_w: np.ndarray) -> np.ndarray:
    """Approximate specific force: a_body = R_w2b @ (dv/dt - g_world)."""
    oid = id(obj)
    prev = _vel_cache.get(oid)
    now = time.monotonic()

    if prev is None:
        _vel_cache[oid] = (lin_vel_w.copy(), now)
        # on first call return gravity vector in body frame
        g_world = np.array([0.0, 0.0, -9.81], dtype=np.float32)
        return _rotate_to_body(q_wxyz, -g_world)

    prev_vel, prev_t = prev
    dt = now - prev_t
    if dt < 1e-6:
        dt = 0.01
    a_world = (lin_vel_w - prev_vel) / dt
    _vel_cache[oid] = (lin_vel_w.copy(), now)

    # subtract gravity
    g_world = np.array([0.0, 0.0, -9.81], dtype=np.float32)
    a_specific_world = a_world - g_world
    return _rotate_to_body(q_wxyz, a_specific_world)
