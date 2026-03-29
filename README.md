<div align="center">
  <h1 align="center">unitree_sim_isaaclab — klapaucjusz setup</h1>
  <p align="center">Local development guide for working with the simulation and on the real robot.</p>
</div>

---

## Installation

```bash
cd ~/
git clone https://github.com/unitreerobotics/unitree_sim_isaaclab.git
cd ~/unitree_sim_isaaclab
chmod +x auto_setup_env.sh
bash auto_setup_env.sh 4.5 unitree_sim_env
```

---

## ROS Domain ID

The `ROS_DOMAIN_ID` must match your target environment. Set it in two places:

### 1. `~/.bashrc`

```bash
export ROS_DOMAIN_ID=1  # simulation = 1, real robot = 0
```

### 2. `.devcontainer/devcontainer.json`

```json
"containerEnv": {
    "ROS_DOMAIN_ID": "1"  // simulation = 1, real robot = 0
}
```

> After changing the devcontainer config, **reopen in container** or **rebuild the container**.

| Environment | `ROS_DOMAIN_ID` |
|-------------|-----------------|
| Simulation  | `1`             |
| Real robot  | `0`             |

---

## Running the Simulation

```bash
cd ~/unitree_sim_isaaclab
conda activate unitree_sim_env

python sim_main.py \
    --device cpu \
    --enable_cameras \
    --task G129-Inspire-Wholebody-Empty-Room \
    --robot_type g129 \
    --enable_inspire_dds \
    --enable_sensor_publisher
```

---

## Running the State Bridge (in container)

> **Note:** The `sim=true` flag is not yet implemented — use the real robot mode for now.

```bash
ros2 launch g1_state_bridge g1_state_bridge.launch.py sim=true
```

---

## Simulation DDS Topics

All topics use the `rt/` prefix (ROS2 convention via DDS). Topics are active on `ROS_DOMAIN_ID=1`.

### Robot state & control

| Topic | Type | Direction |
|-------|------|-----------|
| `rt/lowstate` | `LowState_` | published |
| `rt/lowcmd` | `LowCmd_` | subscribed |
| `rt/odommodestate` | `SportModeState_` | published |

### Hands (Inspire)

| Topic | Type | Direction |
|-------|------|-----------|
| `rt/inspire/state` | `MotorStates_` | published |
| `rt/inspire/cmd` | `MotorCmds_` | subscribed |

### Sim control

| Topic | Type | Direction | Notes |
|-------|------|-----------|-------|
| `rt/sim_state` | `String_` | published | current sim state |
| `rt/sim_state_cmd` | `String_` | subscribed | control commands |
| `rt/rewards_state` | `String_` | published | reward values |
| `rt/rewards_state_cmd` | `String_` | subscribed | reward config |
| `rt/reset_pose/cmd` | `String_` | subscribed | reset robot pose |
| `rt/run_command/cmd` | `String_` | subscribed | wholebody motion cmd |

### Sensors (via `--enable_sensor_publisher`)

| Topic | Type | Direction | Details |
|-------|------|-----------|---------|
| `rt/camera/color/image_raw` | `sensor_msgs/Image` | published | RGB 640×480, `rgb8` |
| `rt/camera/color/camera_info` | `sensor_msgs/CameraInfo` | published | D435i intrinsics |
| `rt/camera/depth/image_rect_raw` | `sensor_msgs/Image` | published | depth 640×480, `32FC1` (metres) |
| `rt/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | published | D435i intrinsics |
| `rt/utlidar/cloud` | `sensor_msgs/PointCloud2` | published | XYZ float32, frame `mid360_link` |
