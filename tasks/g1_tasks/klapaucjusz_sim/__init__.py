# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0

import gymnasium as gym

from . import klapaucjusz_sim_hw_env_cfg


gym.register(
    id="Klapaucjusz-Sim",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": klapaucjusz_sim_hw_env_cfg.KlapaucjuszSimEnvCfg,
    },
    disable_env_checker=True,
)
