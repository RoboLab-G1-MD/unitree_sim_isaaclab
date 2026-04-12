
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
from __future__ import annotations

from typing import TYPE_CHECKING
import numpy as np
import torch
from multiprocessing import shared_memory

from tasks.common_observations.g1_29dof_state import get_robot_boy_joint_states
from tasks.common_observations.inspire_state import get_robot_inspire_joint_states
from tasks.common_observations.camera_state import get_camera_image

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

_LIDAR_SHM_NAME = "isaac_lidar_shm"

_lidar_state = {
    "shm": None,
    "buf": None,
    "placeholder": None,
}

# One entry per depth camera — keyed by scene entity name.
_DEPTH_SHM_NAMES = {
    "depth_camera":      "isaac_depth_shm",       # D435i — obstacle detection
    "head_depth_camera": "isaac_head_depth_shm",  # torso_link — person detection
}

_depth_states = {
    key: {"shm": None, "buf": None, "placeholder": None, "height": None, "width": None}
    for key in _DEPTH_SHM_NAMES
}


def get_lidar_scan(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Read LiDAR hits from the ray caster and write XYZ points to shared memory.

    The shared memory segment ``isaac_lidar_shm`` contains:
      - bytes [0:4]        : uint32  – num_points (N)
      - bytes [4:4+N*12]   : float32[N][3] – XYZ in sensor frame (metres)

    Returns:
        A zero placeholder tensor (the actual data is in shared memory).
    """
    import struct

    lidar_data = env.scene["lidar"].data
    pos_w = lidar_data.pos_w        # (E, 3)
    quat_w = lidar_data.quat_w      # (E, 4) w,x,y,z — full orientation of mid360_link in world frame
    ray_hits_w = lidar_data.ray_hits_w  # (E, N, 3)

    # Vector from sensor to hit in world frame
    vec_w = ray_hits_w[0] - pos_w[0].unsqueeze(0)  # (N, 3)

    # Rotate into mid360_link frame via conjugate quaternion q* = (w, -x, -y, -z)
    w  =  quat_w[0, 0]
    x  = -quat_w[0, 1]  # negated for conjugate
    y  = -quat_w[0, 2]  # negated for conjugate
    z  = -quat_w[0, 3]  # negated for conjugate
    t = 2.0 * torch.stack([
        y * vec_w[:, 2] - z * vec_w[:, 1],
        z * vec_w[:, 0] - x * vec_w[:, 2],
        x * vec_w[:, 1] - y * vec_w[:, 0],
    ], dim=1)
    xyz = (vec_w + w * t + torch.stack([
        y * t[:, 2] - z * t[:, 1],
        z * t[:, 0] - x * t[:, 2],
        x * t[:, 1] - y * t[:, 0],
    ], dim=1)).cpu().numpy().astype(np.float32)  # (N, 3)
    num_points = xyz.shape[0]
    data_bytes = 4 + num_points * 12  # header + N * 3 * float32

    state = _lidar_state
    if state["shm"] is None:
        try:
            state["shm"] = shared_memory.SharedMemory(name=_LIDAR_SHM_NAME)
        except FileNotFoundError:
            state["shm"] = shared_memory.SharedMemory(name=_LIDAR_SHM_NAME, create=True, size=data_bytes)
        state["buf"] = np.ndarray((data_bytes,), dtype=np.uint8, buffer=state["shm"].buf)

    state["buf"][:4] = np.frombuffer(struct.pack("<I", num_points), dtype=np.uint8)
    state["buf"][4:data_bytes] = np.frombuffer(xyz.tobytes(), dtype=np.uint8)

    if state["placeholder"] is None:
        state["placeholder"] = torch.zeros((1,), device=env.device)

    return state["placeholder"]


def get_depth_images(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Read all available depth cameras and write each to its shared memory segment.

    Handles ``depth_camera`` (D435i chest) and ``head_depth_camera`` (head) if present
    in the scene — mirrors the behaviour of ``get_camera_image`` for RGB cameras.

    Shared memory layout per camera (same as before):
      - bytes [0:4]  : uint32 – height
      - bytes [4:8]  : uint32 – width
      - bytes [8:...]: float32 array – depth in metres (row-major, H×W)

    SHM names:
      - ``isaac_depth_shm``      for depth_camera
      - ``isaac_head_depth_shm`` for head_depth_camera

    Returns:
        A zero placeholder tensor.
    """
    import struct

    placeholder = None
    scene_keys = set(env.scene.keys())

    for cam_key, shm_name in _DEPTH_SHM_NAMES.items():
        if cam_key not in scene_keys:
            continue

        depth = env.scene[cam_key].data.output["distance_to_image_plane"]  # (E, H, W, 1)
        depth_np = depth[0, :, :, 0].cpu().numpy().astype(np.float32)      # (H, W)
        h, w = depth_np.shape
        data_bytes = 8 + h * w * 4

        state = _depth_states[cam_key]
        if state["shm"] is None:
            try:
                state["shm"] = shared_memory.SharedMemory(name=shm_name)
            except FileNotFoundError:
                state["shm"] = shared_memory.SharedMemory(name=shm_name, create=True, size=data_bytes)
            state["buf"] = np.ndarray((data_bytes,), dtype=np.uint8, buffer=state["shm"].buf)
            state["height"] = h
            state["width"] = w

        state["buf"][:8] = np.frombuffer(struct.pack("<II", h, w), dtype=np.uint8)
        state["buf"][8:8 + h * w * 4] = np.frombuffer(depth_np.tobytes(), dtype=np.uint8)

        if state["placeholder"] is None:
            state["placeholder"] = torch.zeros((1,), device=env.device)
        placeholder = state["placeholder"]

    if placeholder is None:
        placeholder = torch.zeros((1,), device=env.device)
    return placeholder


# ensure functions can be accessed by external modules
__all__ = [
    "get_robot_boy_joint_states",
    "get_robot_inspire_joint_states",
    "get_lidar_scan",
    "get_camera_image",
    "get_depth_images",
]
