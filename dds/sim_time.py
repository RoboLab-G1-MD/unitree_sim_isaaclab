# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Shared simulation time utility.

All DDS publishers use now_stamp() for message headers.
When use_sim_time=True, sim time is used (requires ClockDDS to be running).
When use_sim_time=False (default), wall clock time is used.
"""

import time

from unitree_sdk2py.idl.builtin_interfaces.msg.dds_ import Time_

_use_sim_time: bool = False
_sim_sec: int = 0
_sim_nanosec: int = 0


def init(use_sim_time: bool) -> None:
    global _use_sim_time
    _use_sim_time = use_sim_time


def update(sec: int, nanosec: int) -> None:
    """Called by ClockDDS on every publish to update current sim time."""
    global _sim_sec, _sim_nanosec
    _sim_sec = sec
    _sim_nanosec = nanosec


def now_stamp() -> Time_:
    """Return current timestamp as Time_ (sim time or wall clock)."""
    if _use_sim_time:
        return Time_(sec=_sim_sec, nanosec=_sim_nanosec)
    now = time.time()
    sec = int(now)
    return Time_(sec=sec, nanosec=int((now - sec) * 1_000_000_000))
