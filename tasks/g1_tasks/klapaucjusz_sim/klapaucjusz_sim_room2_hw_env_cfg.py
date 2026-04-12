# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
import os

project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
_PEOPLE_DIR = os.path.join(project_root, "assets", "humans")

import torch

import isaaclab.envs.mdp as base_mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.markers import VisualizationMarkersCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.sensors.ray_caster import MultiMeshRayCasterCfg  # type: ignore[import]
from isaaclab.sensors.ray_caster.patterns import LidarPatternCfg  # type: ignore[import]
from isaaclab.utils import configclass

from . import mdp
from tasks.common_config import CameraBaseCfg, CameraPresets, G1RobotPresets  # isort: skip
from tasks.common_event.event_manager import SimpleEvent, SimpleEventManager


@configclass
class EmptyRoom15H5SceneCfg(InteractiveSceneCfg):
    """Empty 15m x 15m x 5m room — no cylinder, person in front of robot."""

    robot: ArticulationCfg = G1RobotPresets.g1_29dof_inspire_wholebody(
        init_pos=(0.0, 0.0, 0.80),
        init_rot=(1, 0, 0, 0),
    )

    contact_forces = ContactSensorCfg(
        prim_path="/World/envs/env_.*/Robot/.*",
        history_length=10,
        track_air_time=True,
        debug_vis=False,
    )

    room_floor = AssetBaseCfg(
        prim_path="/World/envs/env_.*/RoomFloor",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.0, 0.0, -0.05], rot=[1.0, 0.0, 0.0, 0.0]),
        spawn=sim_utils.CuboidCfg(
            size=(15.0, 15.0, 0.1),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.52, 0.52, 0.52), metallic=0.0),
        ),
    )

    room_wall_north = AssetBaseCfg(
        prim_path="/World/envs/env_.*/RoomWallNorth",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.0, 7.55, 2.5], rot=[1.0, 0.0, 0.0, 0.0]),
        spawn=sim_utils.CuboidCfg(
            size=(15.0, 0.1, 5.0),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.62, 0.62, 0.62), metallic=0.0),
        ),
    )

    room_wall_south = AssetBaseCfg(
        prim_path="/World/envs/env_.*/RoomWallSouth",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.0, -7.55, 2.5], rot=[1.0, 0.0, 0.0, 0.0]),
        spawn=sim_utils.CuboidCfg(
            size=(15.0, 0.1, 5.0),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.62, 0.62, 0.62), metallic=0.0),
        ),
    )

    room_wall_east = AssetBaseCfg(
        prim_path="/World/envs/env_.*/RoomWallEast",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[7.55, 0.0, 2.5], rot=[1.0, 0.0, 0.0, 0.0]),
        spawn=sim_utils.CuboidCfg(
            size=(0.1, 15.0, 5.0),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.62, 0.62, 0.62), metallic=0.0),
        ),
    )

    room_wall_west = AssetBaseCfg(
        prim_path="/World/envs/env_.*/RoomWallWest",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[-7.55, 0.0, 2.5], rot=[1.0, 0.0, 0.0, 0.0]),
        spawn=sim_utils.CuboidCfg(
            size=(0.1, 15.0, 5.0),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.62, 0.62, 0.62), metallic=0.0),
        ),
    )

    # Lidar sees only floor and walls — no cylinder in this room.
    lidar = MultiMeshRayCasterCfg(
        prim_path="/World/envs/env_.*/Robot/mid360_link",
        ray_alignment="base",
        mesh_prim_paths=[
            "/World/envs/env_.*/RoomFloor",
            "/World/envs/env_.*/RoomWallNorth",
            "/World/envs/env_.*/RoomWallSouth",
            "/World/envs/env_.*/RoomWallEast",
            "/World/envs/env_.*/RoomWallWest",
            "/World/envs/env_.*/Person",
            # "/World/envs/env_.*/PersonLeft",
            # "/World/envs/env_.*/PersonRight",
        ],
        pattern_cfg=LidarPatternCfg(
            channels=40,
            vertical_fov_range=(-7.0, 52.0),
            horizontal_fov_range=(-180.0, 180.0),
            horizontal_res=1.0,
        ),
        offset=MultiMeshRayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 0.0), rot=(0.0, 0.0, 1.0, 0.0)),
        max_distance=40.0,
        debug_vis=True,
        visualizer_cfg=VisualizationMarkersCfg(
            prim_path="/Visuals/RayCaster",
            markers={
                "hit": sim_utils.SphereCfg(
                    radius=0.01,
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
                ),
            },
        ),
    )

    # Static NVIDIA Isaac Sim characters for person detection testing (YOLO).
    # Run fetch_people.sh once to download assets from NVIDIA's public S3.
    person = AssetBaseCfg(
        prim_path="/World/envs/env_.*/Person",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[2.5, 0.0, 0.0], rot=[0.7071, 0.0, 0.0, -0.7071]),
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{_PEOPLE_DIR}/male_construction/male_construction.usd",
        ),
    )
    # person_left = AssetBaseCfg(
    #     prim_path="/World/envs/env_.*/PersonLeft",
    #     init_state=AssetBaseCfg.InitialStateCfg(pos=[0.0, 2.5, 0.0], rot=[0.7071, 0.0, 0.0, 0.0]),
    #     spawn=sim_utils.UsdFileCfg(
    #         usd_path=f"{_PEOPLE_DIR}/female_business/female_business.usd",
    #     ),
    # )
    # person_right = AssetBaseCfg(
    #     prim_path="/World/envs/env_.*/PersonRight",
    #     init_state=AssetBaseCfg.InitialStateCfg(pos=[0.0, -2.5, 0.0], rot=[0.0, 0.0, 0.0, 1.0]),
    #     spawn=sim_utils.UsdFileCfg(
    #         usd_path=f"{_PEOPLE_DIR}/female_medical/female_medical.usd",
    #     ),
    # )

    # ── cameras ───────────────────────────────────────────────────────────
    # Head RGB  (torso_link): 640x480, HFOV≈69° — person detection
    head_color_camera = CameraPresets.g1_head_color_camera()
    # Head depth (torso_link): 640x480, HFOV≈87° — person detection
    head_depth_camera = CameraPresets.g1_head_depth_camera()
    # D435i RGB  (d435_link): 640x480, HFOV≈69° — obstacle detection
    # color_camera = CameraPresets.g1_color_camera()
    # D435i depth (d435_link): 640x480, HFOV≈87° — obstacle detection
    # depth_camera = CameraPresets.g1_depth_camera()

    robot_camera = CameraPresets.g1_world_camera()

    world_camera = CameraBaseCfg.get_camera_config(
        prim_path="/World/PerspectiveCamera",
        pos_offset=(-1.9, -5.0, 1.8),
        rot_offset=(-0.40614, 0.78544, 0.4277, -0.16986),
    )

    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=2400.0),
    )


@configclass
class ActionsCfg:
    joint_pos = mdp.JointPositionActionCfg(asset_name="robot", joint_names=[".*"], scale=1.0, use_default_offset=True)


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        robot_joint_state = ObsTerm(func=mdp.get_robot_boy_joint_states)
        robot_inspire_state = ObsTerm(func=mdp.get_robot_inspire_joint_states)
        lidar_scan = ObsTerm(func=mdp.get_lidar_scan)
        camera_image = ObsTerm(func=mdp.get_camera_image)
        depth_images = ObsTerm(func=mdp.get_depth_images)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    policy: PolicyCfg = PolicyCfg()


@configclass
class TerminationsCfg:
    pass


@configclass
class RewardsCfg:
    pass


@configclass
class EventCfg:
    pass


@configclass
class KlapaucjuszRoom2EnvCfg(ManagerBasedRLEnvCfg):
    scene: EmptyRoom15H5SceneCfg = EmptyRoom15H5SceneCfg(num_envs=1, env_spacing=2.5, replicate_physics=True)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events = EventCfg()
    commands = None
    rewards: RewardsCfg = RewardsCfg()
    curriculum = None

    def __post_init__(self):
        self.decimation = 4
        self.episode_length_s = 20.0
        self.sim.dt = 0.005
        self.scene.contact_forces.update_period = self.sim.dt
        self.sim.render_interval = self.decimation
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625
        self.sim.physics_material.static_friction = 1.0
        self.sim.physics_material.dynamic_friction = 1.0
        self.sim.physics_material.friction_combine_mode = "max"
        self.sim.physics_material.restitution_combine_mode = "max"
        self.event_manager = SimpleEventManager()

        self.event_manager.register(
            "reset_object_self",
            SimpleEvent(func=lambda env: None),
        )
        self.event_manager.register(
            "reset_all_self",
            SimpleEvent(
                func=lambda env: base_mdp.reset_scene_to_default(
                    env,
                    torch.arange(env.num_envs, device=env.device),
                )
            ),
        )
