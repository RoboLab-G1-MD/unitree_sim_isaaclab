# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
import torch

import isaaclab.envs.mdp as base_mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.markers import VisualizationMarkersCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.sensors.ray_caster import MultiMeshRayCasterCfg  # type: ignore[import]
from isaaclab.sensors.ray_caster.patterns import LidarPatternCfg  # type: ignore[import]
from isaaclab.utils import configclass

from . import mdp
from tasks.common_config import CameraBaseCfg, CameraPresets, G1RobotPresets  # isort: skip
from tasks.common_event.event_manager import SimpleEvent, SimpleEventManager


@configclass
class EmptyRoom15SceneCfg(InteractiveSceneCfg):
    """Empty 15m x 15m x 3m room scene for g1 inspire wholebody."""

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
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.0, 7.55, 1.5], rot=[1.0, 0.0, 0.0, 0.0]),
        spawn=sim_utils.CuboidCfg(
            size=(15.0, 0.1, 3.0),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.62, 0.62, 0.62), metallic=0.0),
        ),
    )

    room_wall_south = AssetBaseCfg(
        prim_path="/World/envs/env_.*/RoomWallSouth",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.0, -7.55, 1.5], rot=[1.0, 0.0, 0.0, 0.0]),
        spawn=sim_utils.CuboidCfg(
            size=(15.0, 0.1, 3.0),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.62, 0.62, 0.62), metallic=0.0),
        ),
    )

    room_wall_east = AssetBaseCfg(
        prim_path="/World/envs/env_.*/RoomWallEast",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[7.55, 0.0, 1.5], rot=[1.0, 0.0, 0.0, 0.0]),
        spawn=sim_utils.CuboidCfg(
            size=(0.1, 15.0, 3.0),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.62, 0.62, 0.62), metallic=0.0),
        ),
    )

    room_wall_west = AssetBaseCfg(
        prim_path="/World/envs/env_.*/RoomWallWest",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[-7.55, 0.0, 1.5], rot=[1.0, 0.0, 0.0, 0.0]),
        spawn=sim_utils.CuboidCfg(
            size=(0.1, 15.0, 3.0),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.62, 0.62, 0.62), metallic=0.0),
        ),
    )

    robot: ArticulationCfg = G1RobotPresets.g1_29dof_inspire_wholebody(
        init_pos=(0.0, 0.0, 0.80),
        init_rot=(1, 0, 0, 0),
    )

    contact_forces = ContactSensorCfg(prim_path="/World/envs/env_.*/Robot/.*", history_length=10, track_air_time=True, debug_vis=False)

    lidar = MultiMeshRayCasterCfg(
        prim_path="/World/envs/env_.*/Robot/mid360_link",
        mesh_prim_paths=[
            "/World/envs/env_.*/RoomFloor",
            "/World/envs/env_.*/RoomWallNorth",
            "/World/envs/env_.*/RoomWallSouth",
            "/World/envs/env_.*/RoomWallEast",
            "/World/envs/env_.*/RoomWallWest",
            "/World/envs/env_.*/Cylinder",
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

    cylinder = AssetBaseCfg(
        prim_path="/World/envs/env_.*/Cylinder",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[2.0, 0.0, 0.5], rot=[1.0, 0.0, 0.0, 0.0]),
        spawn=sim_utils.CylinderCfg(
            radius=0.2,
            height=1.0,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.2, 0.2), metallic=0.0),
        ),
    )

    # D435i RGB: 640x480, HFOV=69°, VFOV=42°
    front_camera = CameraBaseCfg.get_camera_config(
        prim_path="/World/envs/env_.*/Robot/d435_link/front_cam",
        height=480,
        width=640,
        focal_length=7.6,
        horizontal_aperture=10.46,
        clipping_range=(0.1, 10.0),
        data_types=["rgb"],
        rot_offset=(0.5, -0.5, 0.5, -0.5),
    )
    # D435i depth: 640x480, HFOV=87°, zasięg 0.1–10m
    depth_camera = CameraBaseCfg.get_camera_config(
        prim_path="/World/envs/env_.*/Robot/d435_link/depth_cam",
        height=480,
        width=640,
        focal_length=7.6,
        horizontal_aperture=14.42,
        clipping_range=(0.1, 10.0),
        data_types=["distance_to_image_plane"],
        rot_offset=(0.5, -0.5, 0.5, -0.5),
    )
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
        camera_image = ObsTerm(func=mdp.get_camera_image)
        lidar_scan = ObsTerm(func=mdp.get_lidar_scan)
        depth_image = ObsTerm(func=mdp.get_depth_image)

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
class KlapaucjuszSimEnvCfg(ManagerBasedRLEnvCfg):
    scene: EmptyRoom15SceneCfg = EmptyRoom15SceneCfg(num_envs=1, env_spacing=2.5, replicate_physics=True)
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
