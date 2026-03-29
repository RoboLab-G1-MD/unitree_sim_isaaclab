# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache-2.0
"""CycloneDDS IDL definitions for sensor_msgs/Image and sensor_msgs/CameraInfo.

Type names use dot notation matching unitree_sdk2py convention, which CycloneDDS
maps to the same wire format as ROS2's double-colon notation.
"""

from dataclasses import dataclass, field
from typing import List

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types

from unitree_sdk2py.idl.std_msgs.msg.dds_ import Header_


@dataclass
@annotate.final
@annotate.autoid("sequential")
class RegionOfInterest_(idl.IdlStruct, typename="sensor_msgs.msg.dds_.RegionOfInterest_"):
    x_offset: types.uint32
    y_offset: types.uint32
    height: types.uint32
    width: types.uint32
    do_rectify: bool


@dataclass
@annotate.final
@annotate.autoid("sequential")
class Image_(idl.IdlStruct, typename="sensor_msgs.msg.dds_.Image_"):
    header: Header_
    height: types.uint32
    width: types.uint32
    encoding: str
    is_bigendian: types.uint8
    step: types.uint32                    # row length in bytes
    data: types.sequence[types.uint8]    # pixel data


@dataclass
@annotate.final
@annotate.autoid("sequential")
class CameraInfo_(idl.IdlStruct, typename="sensor_msgs.msg.dds_.CameraInfo_"):
    header: Header_
    height: types.uint32
    width: types.uint32
    distortion_model: str
    d: types.sequence[types.float64]      # distortion coefficients
    k: types.array[types.float64, 9]      # 3×3 camera matrix (row-major)
    r: types.array[types.float64, 9]      # 3×3 rectification matrix
    p: types.array[types.float64, 12]     # 3×4 projection matrix
    binning_x: types.uint32
    binning_y: types.uint32
    roi: RegionOfInterest_
