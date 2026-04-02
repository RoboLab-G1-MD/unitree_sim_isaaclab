# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""OdomStateDDS — publishes rt/dog_odom (nav_msgs/Odometry) from Isaac Lab ground-truth.

Published fields:
  pose.position        – XYZ in world/odom frame (metres)
  pose.orientation     – quaternion (x,y,z,w)
  twist.linear         – linear velocity in world frame (m/s)
  twist.angular        – angular velocity in world frame (rad/s)

frame_id: odom  |  child_frame_id: base_link
"""

import numpy as np

import dds.sim_time as sim_time

from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import (
    PoseWithCovariance_, TwistWithCovariance_,
    Pose_, Twist_, Point_, Quaternion_, Vector3_,
)
from unitree_sdk2py.idl.std_msgs.msg.dds_ import Header_
from unitree_sdk2py.idl.builtin_interfaces.msg.dds_ import Time_

from dds.dds_base import DDSObject

_TOPIC = "rt/dog_odom"


class OdomStateDDS(DDSObject):
    """Publishes ground-truth odometry from Isaac Lab as nav_msgs/Odometry on rt/dog_odom."""

    node_name = "OdomStateDDS"

    def __init__(self, env):
        super().__init__()
        self._env = env
        self._publisher = None
        self._msg = None

    def setup_publisher(self):
        self._publisher = ChannelPublisher(_TOPIC, Odometry_)
        self._publisher.Init()

        # Diagonal covariance: ground-truth pose is very accurate (low uncertainty)
        pose_cov = [0.0] * 36
        pose_cov[0]  = 0.01   # x
        pose_cov[7]  = 0.01   # y
        pose_cov[14] = 0.01   # z
        pose_cov[21] = 0.05   # roll
        pose_cov[28] = 0.05   # pitch
        pose_cov[35] = 0.05   # yaw

        twist_cov = [0.0] * 36
        twist_cov[0]  = 0.05  # vx
        twist_cov[7]  = 0.05  # vy
        twist_cov[14] = 0.05  # vz
        twist_cov[21] = 0.1   # vroll
        twist_cov[28] = 0.1   # vpitch
        twist_cov[35] = 0.1   # vyaw

        self._msg = Odometry_(
            header=Header_(stamp=Time_(sec=0, nanosec=0), frame_id="odom"),
            child_frame_id="base_link",
            pose=PoseWithCovariance_(
                pose=Pose_(
                    position=Point_(x=0.0, y=0.0, z=0.0),
                    orientation=Quaternion_(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
                covariance=pose_cov,
            ),
            twist=TwistWithCovariance_(
                twist=Twist_(
                    linear=Vector3_(x=0.0, y=0.0, z=0.0),
                    angular=Vector3_(x=0.0, y=0.0, z=0.0),
                ),
                covariance=twist_cov,
            ),
        )
        print(f"[{self.node_name}] Publisher ready on {_TOPIC}")

    def setup_subscriber(self):
        pass

    def dds_subscriber(self, msg, datatype=None):
        pass

    def dds_publisher(self):
        """Read ground-truth pose/velocity from Isaac Lab and publish nav_msgs/Odometry."""
        try:
            data = self._env.scene["robot"].data

            pos      = data.root_pos_w[0].cpu().numpy()       # (3,) [x,y,z]
            quat     = data.root_quat_w[0].cpu().numpy()      # (4,) [w,x,y,z] Isaac Lab w-first
            lin_vel  = data.root_lin_vel_w[0].cpu().numpy()   # (3,)
            ang_vel  = data.root_ang_vel_w[0].cpu().numpy()   # (3,)

            stamp = sim_time.now_stamp()
            msg = self._msg
            msg.header.stamp.sec     = stamp.sec
            msg.header.stamp.nanosec = stamp.nanosec

            # position
            msg.pose.pose.position.x = float(pos[0])
            msg.pose.pose.position.y = float(pos[1])
            msg.pose.pose.position.z = float(pos[2])

            # orientation — convert Isaac Lab [w,x,y,z] → ROS [x,y,z,w]
            msg.pose.pose.orientation.x = float(quat[1])
            msg.pose.pose.orientation.y = float(quat[2])
            msg.pose.pose.orientation.z = float(quat[3])
            msg.pose.pose.orientation.w = float(quat[0])

            # linear velocity (world frame)
            msg.twist.twist.linear.x = float(lin_vel[0])
            msg.twist.twist.linear.y = float(lin_vel[1])
            msg.twist.twist.linear.z = float(lin_vel[2])

            # angular velocity (world frame)
            msg.twist.twist.angular.x = float(ang_vel[0])
            msg.twist.twist.angular.y = float(ang_vel[1])
            msg.twist.twist.angular.z = float(ang_vel[2])

            self._publisher.Write(msg)
        except Exception as e:
            print(f"[{self.node_name}] publish error: {e}")
