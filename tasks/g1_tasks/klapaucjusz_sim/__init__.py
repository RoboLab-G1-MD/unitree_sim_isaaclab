# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0

import gymnasium as gym

from . import klapaucjusz_sim_hw_env_cfg, klapaucjusz_sim_room2_hw_env_cfg


gym.register(
    id="Klapaucjusz-Sim",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": klapaucjusz_sim_hw_env_cfg.KlapaucjuszSimEnvCfg,
    },
    disable_env_checker=True,
)

gym.register(
    id="Klapaucjusz-Sim-Room2",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": klapaucjusz_sim_room2_hw_env_cfg.KlapaucjuszRoom2EnvCfg,
    },
    disable_env_checker=True,
)
