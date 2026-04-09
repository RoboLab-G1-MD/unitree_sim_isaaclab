
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
_DEPTH_SHM_NAME = "isaac_depth_shm"

_lidar_state = {
    "shm": None,
    "buf": None,
    "placeholder": None,
}

_depth_state = {
    "shm": None,
    "buf": None,
    "placeholder": None,
    "height": None,
    "width": None,
}

_HEAD_DEPTH_SHM_NAME = "isaac_head_depth_shm"

_head_depth_state = {
    "shm": None,
    "buf": None,
    "placeholder": None,
    "height": None,
    "width": None,
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
    ray_hits_w = lidar_data.ray_hits_w  # (E, N, 3)

    # XYZ relative to sensor origin, env 0 only
    xyz = (ray_hits_w[0] - pos_w[0].unsqueeze(0)).cpu().numpy().astype(np.float32)  # (N, 3)
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


def get_depth_image(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Read D435i depth image and write to shared memory.

    The shared memory segment ``isaac_depth_shm`` contains:
      - bytes [0:4]  : uint32 – height
      - bytes [4:8]  : uint32 – width
      - bytes [8:...]: float32 array – depth in metres (row-major, shape H×W)

    Returns:
        A zero placeholder tensor (the actual data is in shared memory).
    """
    import struct

    depth = env.scene["depth_camera"].data.output["distance_to_image_plane"]  # (E, H, W, 1)
    depth_np = depth[0, :, :, 0].cpu().numpy().astype(np.float32)  # (H, W)
    h, w = depth_np.shape
    data_bytes = 8 + h * w * 4  # 8-byte header + float32 pixels

    state = _depth_state
    if state["shm"] is None:
        try:
            state["shm"] = shared_memory.SharedMemory(name=_DEPTH_SHM_NAME)
        except FileNotFoundError:
            state["shm"] = shared_memory.SharedMemory(name=_DEPTH_SHM_NAME, create=True, size=data_bytes)
        state["buf"] = np.ndarray((data_bytes,), dtype=np.uint8, buffer=state["shm"].buf)
        state["height"] = h
        state["width"] = w

    state["buf"][:8] = np.frombuffer(struct.pack("<II", h, w), dtype=np.uint8)
    state["buf"][8:8 + h * w * 4] = np.frombuffer(depth_np.tobytes(), dtype=np.uint8)

    if state["placeholder"] is None:
        state["placeholder"] = torch.zeros((1,), device=env.device)

    return state["placeholder"]


def get_head_depth_image(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Read head front camera depth image and write to shared memory.

    The shared memory segment ``isaac_head_depth_shm`` contains:
      - bytes [0:4]  : uint32 – height
      - bytes [4:8]  : uint32 – width
      - bytes [8:...]: float32 array – depth in metres (row-major, shape H×W)

    Returns:
        A zero placeholder tensor (the actual data is in shared memory).
    """
    import struct

    depth = env.scene["head_depth_camera"].data.output["distance_to_image_plane"]  # (E, H, W, 1)
    depth_np = depth[0, :, :, 0].cpu().numpy().astype(np.float32)  # (H, W)
    h, w = depth_np.shape
    data_bytes = 8 + h * w * 4

    state = _head_depth_state
    if state["shm"] is None:
        try:
            state["shm"] = shared_memory.SharedMemory(name=_HEAD_DEPTH_SHM_NAME)
        except FileNotFoundError:
            state["shm"] = shared_memory.SharedMemory(name=_HEAD_DEPTH_SHM_NAME, create=True, size=data_bytes)
        state["buf"] = np.ndarray((data_bytes,), dtype=np.uint8, buffer=state["shm"].buf)
        state["height"] = h
        state["width"] = w

    state["buf"][:8] = np.frombuffer(struct.pack("<II", h, w), dtype=np.uint8)
    state["buf"][8:8 + h * w * 4] = np.frombuffer(depth_np.tobytes(), dtype=np.uint8)

    if state["placeholder"] is None:
        state["placeholder"] = torch.zeros((1,), device=env.device)

    return state["placeholder"]


# ensure functions can be accessed by external modules
__all__ = [
    "get_robot_boy_joint_states",
    "get_robot_inspire_joint_states",
    "get_camera_image",
    "get_lidar_scan",
    "get_depth_image",
    "get_head_depth_image",
]

