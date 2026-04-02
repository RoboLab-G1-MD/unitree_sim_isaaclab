# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""ClockDDS — publishes rt/clock (rosgraph_msgs/Clock) from Isaac Lab simulation time.

Enables use_sim_time: true in ROS2 nodes. Updated the shared sim_time module
so all other DDS publishers use consistent sim timestamps.
"""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate

from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.builtin_interfaces.msg.dds_ import Time_

import dds.sim_time as sim_time
from dds.dds_base import DDSObject

_TOPIC = "rt/clock"


@dataclass
@annotate.final
@annotate.autoid("sequential")
class Clock_(idl.IdlStruct, typename="rosgraph_msgs.msg.dds_.Clock_"):
    clock: Time_


class ClockDDS(DDSObject):
    """Publishes rosgraph_msgs/Clock on rt/clock from Isaac Lab sim time."""

    node_name = "ClockDDS"

    def __init__(self, env):
        super().__init__()
        self._env = env
        self._publisher = None
        self._msg = None

    def setup_publisher(self):
        self._publisher = ChannelPublisher(_TOPIC, Clock_)
        self._publisher.Init()
        self._msg = Clock_(clock=Time_(sec=0, nanosec=0))
        print(f"[{self.node_name}] Publisher ready on {_TOPIC}")

    def setup_subscriber(self):
        pass

    def dds_subscriber(self, msg, datatype=None):
        pass

    def dds_publisher(self):
        try:
            t = float(self._env.sim.current_time)
            sec = int(t)
            nanosec = int((t - sec) * 1_000_000_000)
            sim_time.update(sec, nanosec)
            self._msg.clock.sec = sec
            self._msg.clock.nanosec = nanosec
            self._publisher.Write(self._msg)
        except Exception as e:
            print(f"[{self.node_name}] publish error: {e}")
