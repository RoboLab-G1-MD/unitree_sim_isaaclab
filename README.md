<div align="center">
  <h1 align="center">unitree_sim_isaaclab — Klapaucjusz fork</h1>
  <p align="center">Isaac Lab simulation for Unitree G1 with LiDAR, cameras and ROS2 integration.</p>
  <p align="center">
    Fork of <a href="https://github.com/unitreerobotics/unitree_sim_isaaclab">unitree_sim_isaaclab</a> by Unitree Robotics.
    See <a href="README_upstream.md">README_upstream.md</a> for original docs.
  </p>
</div>

---

## Installation

```bash
cd ~/
git clone https://github.com/RoboLab-G1-MD/unitree_sim_isaaclab.git
cd ~/unitree_sim_isaaclab
chmod +x auto_setup_env.sh
bash auto_setup_env.sh 4.5 unitree_sim_env
```

---

## ROS Domain ID

| Environment | `ROS_DOMAIN_ID` |
|-------------|-----------------|
| Simulation  | `1`             |
| Real robot  | `0`             |

Set in two places before starting:

**`~/.bashrc`** (on the host laptop):
```bash
export ROS_DOMAIN_ID=1  # simulation = 1, real robot = 0
```

**`klapaucjusz_ros/.devcontainer/devcontainer.json`**:
```json
"containerEnv": {
    "ROS_DOMAIN_ID": "1"  // simulation = 1, real robot = 0
}
```

After changing devcontainer config — **Reopen in Container** or **Rebuild**.

---

## Running the Simulation

```bash
cd ~/unitree_sim_isaaclab
conda activate unitree_sim_env

python sim_main.py \
    --device cpu \
    --enable_cameras \
    --task Klapaucjusz-Sim \
    --robot_type g129 \
    --enable_inspire_dds \
    --enable_sensor_publisher
```

---

## Running the State Bridge in klapaucjusz_ros (in devcontainer)

> **Note:** The `sim=true` flag is not yet implemented — use the real robot mode for now.

```bash
ros2 launch g1_state_bridge g1_state_bridge.launch.py sim=true
```

---

## DDS Topics (ROS_DOMAIN_ID=1)

All topics use the `rt/` prefix — visible in ROS2 as topics without `rt/`.

### Robot state & control

| DDS topic | ROS2 topic | Type | Direction |
|-----------|------------|------|-----------|
| `rt/lowstate` | `/lowstate` | `unitree_hg/LowState` | sim → ROS2 |
| `rt/lowcmd` | `/lowcmd` | `unitree_hg/LowCmd` | ROS2 → sim |
| `rt/odommodestate` | `/odommodestate` | `unitree_go/SportModeState` | sim → ROS2 |

### Hands (Inspire)

| DDS topic | ROS2 topic | Direction |
|-----------|------------|-----------|
| `rt/inspire/state` | `/inspire/state` | sim → ROS2 |
| `rt/inspire/cmd` | `/inspire/cmd` | ROS2 → sim |

### Sim control

| DDS topic | ROS2 topic | Direction | Notes |
|-----------|------------|-----------|-------|
| `rt/sim_state` | `/sim_state` | sim → ROS2 | current sim state (JSON) |
| `rt/rewards_state` | `/rewards_state` | sim → ROS2 | reward values (JSON) |
| `rt/reset_pose/cmd` | `/reset_pose/cmd` | ROS2 → sim | reset robot pose |
| `rt/run_command/cmd` | `/run_command/cmd` | ROS2 → sim | wholebody motion cmd |

### Sensors (`--enable_sensor_publisher`)

| ROS2 topic | Type | Details |
|------------|------|---------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB 640×480, `rgb8` |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | D435i intrinsics (fx≈465 px) |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | depth 640×480, `32FC1` (metres) |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | D435i depth intrinsics (fx≈337 px) |
| `/utlidar/cloud` | `sensor_msgs/PointCloud2` | Livox Mid-360, XYZ float32, frame `mid360_link` |

### State bridge output (after `g1_state_bridge`)

| ROS2 topic | Type |
|------------|------|
| `/joint_states` | `sensor_msgs/JointState` |
| `/imu/data` | `sensor_msgs/Imu` |
| `/odom/legs` | `nav_msgs/Odometry` |
| `/tf` | TF tree: `odom → base_link → ... → mid360_link` |

---

## What was added in this fork

- **`tasks/g1_tasks/klapaucjusz_sim/`** — empty room task with LiDAR + D435i cameras + cylinder obstacle
- **`dds/lidar_dds.py`** — publishes Livox Mid-360 point cloud via DDS
- **`dds/camera_dds.py`** — publishes RGB + depth images + CameraInfo via DDS
- **`dds/odom_state_dds.py`** — publishes ground-truth odometry as `SportModeState`
- **`dds/ros2_image_msgs.py`** — CycloneDDS IDL types for `sensor_msgs/Image` and `CameraInfo`
- **`sensor_publisher.py`** — background threads for LiDAR + camera DDS publishing
