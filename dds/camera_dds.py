# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache-2.0
"""CameraDDS — publishes RGB + depth images and CameraInfo via DDS (ROS2 domain 1).

Topics published (rt/ prefix → ROS2 topic without prefix):
  rt/camera/color/image_raw       sensor_msgs/Image   (rgb8, 640×480)
  rt/camera/color/camera_info     sensor_msgs/CameraInfo
  rt/camera/depth/image_rect_raw  sensor_msgs/Image   (32FC1 metres, 640×480)
  rt/camera/depth/camera_info     sensor_msgs/CameraInfo

D435i intrinsics are derived from Isaac Lab camera config:
  RGB:   focal_length=7.6, horizontal_aperture=10.46  → fx=fy≈465.4 px
  Depth: focal_length=7.6, horizontal_aperture=14.42  → fx=fy≈337.3 px
"""

import numpy as np

import dds.sim_time as sim_time

from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.std_msgs.msg.dds_ import Header_
from unitree_sdk2py.idl.builtin_interfaces.msg.dds_ import Time_

from dds.ros2_image_msgs import Image_, CameraInfo_, RegionOfInterest_

# ── camera parameters (D435i) ──────────────────────────────────────────────
_H, _W = 480, 640
_CX, _CY = _W / 2.0, _H / 2.0

def _fx(horizontal_aperture: float) -> float:
    return _W * 7.6 / horizontal_aperture

_FX_RGB   = _fx(10.46)   # ≈ 465.4 px
_FX_DEPTH = _fx(14.42)   # ≈ 337.3 px

# ── topic names ────────────────────────────────────────────────────────────
_TOPIC_RGB       = "rt/camera/color/image_raw"
_TOPIC_RGB_INFO  = "rt/camera/color/camera_info"
_TOPIC_DEPTH     = "rt/camera/depth/image_rect_raw"
_TOPIC_DEPTH_INFO = "rt/camera/depth/camera_info"

_FRAME_COLOR = "d435_optical_link"
_FRAME_DEPTH = "d435_optical_link"


class CameraDDS:
    """Publishes RGB and depth camera data as ROS2-compatible DDS topics."""

    def __init__(self):
        self._pub_rgb        = None
        self._pub_rgb_info   = None
        self._pub_depth      = None
        self._pub_depth_info = None

        self._info_color = _build_camera_info(_FX_RGB,   _FRAME_COLOR)
        self._info_depth = _build_camera_info(_FX_DEPTH, _FRAME_DEPTH)

    def setup_publishers(self) -> None:
        self._pub_rgb        = ChannelPublisher(_TOPIC_RGB,        Image_)
        self._pub_rgb_info   = ChannelPublisher(_TOPIC_RGB_INFO,   CameraInfo_)
        self._pub_depth      = ChannelPublisher(_TOPIC_DEPTH,      Image_)
        self._pub_depth_info = ChannelPublisher(_TOPIC_DEPTH_INFO, CameraInfo_)

        for pub in (self._pub_rgb, self._pub_rgb_info,
                    self._pub_depth, self._pub_depth_info):
            pub.Init()

        print(f"[CameraDDS] Publishers ready: {_TOPIC_RGB}, {_TOPIC_DEPTH}")

    def publish_color(self, bgr_np: np.ndarray) -> None:
        """Publish RGB image from a BGR uint8 numpy array (H, W, 3) or (H, W, 4)."""
        if bgr_np is None or self._pub_rgb is None:
            return
        # Isaac Lab outputs RGBA; drop alpha if present
        if bgr_np.ndim == 3 and bgr_np.shape[2] == 4:
            bgr_np = bgr_np[:, :, :3]
        # Convert BGR → RGB
        rgb = bgr_np[:, :, ::-1].astype(np.uint8)

        stamp = _now_stamp()
        msg = Image_(
            header=Header_(stamp=stamp, frame_id=_FRAME_COLOR),
            height=rgb.shape[0],
            width=rgb.shape[1],
            encoding="rgb8",
            is_bigendian=0,
            step=rgb.shape[1] * 3,
            data=rgb.tobytes(),
        )
        self._pub_rgb.Write(msg)

        self._info_color.header.stamp = stamp
        self._pub_rgb_info.Write(self._info_color)

    def publish_depth(self, depth_np: np.ndarray) -> None:
        """Publish depth image from a float32 numpy array (H, W) in metres."""
        if depth_np is None or self._pub_depth is None:
            return
        depth = depth_np.astype(np.float32)

        stamp = _now_stamp()
        msg = Image_(
            header=Header_(stamp=stamp, frame_id=_FRAME_DEPTH),
            height=depth.shape[0],
            width=depth.shape[1],
            encoding="32FC1",
            is_bigendian=0,
            step=depth.shape[1] * 4,
            data=depth.tobytes(),
        )
        self._pub_depth.Write(msg)

        self._info_depth.header.stamp = stamp
        self._pub_depth_info.Write(self._info_depth)


# ── helpers ────────────────────────────────────────────────────────────────

def _now_stamp() -> Time_:
    return sim_time.now_stamp()


def _build_camera_info(fx: float, frame_id: str) -> CameraInfo_:
    """Build a static CameraInfo_ for a pinhole camera with square pixels."""
    fy = fx
    k = [fx, 0.0, _CX,
         0.0, fy, _CY,
         0.0, 0.0, 1.0]
    r = [1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0]
    p = [fx, 0.0, _CX, 0.0,
         0.0, fy,  _CY, 0.0,
         0.0, 0.0, 1.0, 0.0]
    roi = RegionOfInterest_(x_offset=0, y_offset=0,
                            height=0, width=0, do_rectify=False)
    return CameraInfo_(
        header=Header_(stamp=Time_(sec=0, nanosec=0), frame_id=frame_id),
        height=_H,
        width=_W,
        distortion_model="plumb_bob",
        d=[0.0, 0.0, 0.0, 0.0, 0.0],
        k=k,
        r=r,
        p=p,
        binning_x=0,
        binning_y=0,
        roi=roi,
    )
