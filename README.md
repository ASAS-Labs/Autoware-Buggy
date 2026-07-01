# Autoware Universe — S-AM Gokart Platform

This repository contains all configuration, custom nodes, and documentation for
running [Autoware Universe](https://github.com/autowarefoundation/autoware) on the
S-AM gokart platform: a Jetson AGX Orin paired with a single Ouster OS0-32 LiDAR,
no GNSS, no camera-based perception (yet), and a custom vehicle interface.

> **This repo does not ship Autoware itself.** Follow every step below in order —
> the guide walks through the full Autoware source installation and then applies the
> gokart-specific changes on top of it in one continuous flow.

---

## ⚠️ Jetson vs Laptop — Architecture Differences

The installation steps in this README are written for the **Jetson AGX Orin
(ARM64 / aarch64)**. If you are setting up Autoware on an **x86_64 laptop or
desktop** instead (e.g. for simulation, development, or testing with a bag file),
several steps differ.

| | Jetson AGX Orin | x86_64 Laptop / Desktop |
|---|---|---|
| Architecture | `aarch64` | `x86_64` |
| OS | Ubuntu 22.04 + JetPack 6.2.2 | Ubuntu 22.04 |
| CUDA install | Bundled with JetPack — already present | Install from [NVIDIA CUDA Toolkit](https://developer.nvidia.com/cuda-downloads) separately |
| CUDA compute capability flag | `-DCUDA_ARCH_BIN="8.7"` (Ampere, Orin) | Depends on GPU — e.g. `8.6` for RTX 3000 series, `8.9` for RTX 4000 series |
| PyTorch install | JetPack-specific wheel from NVIDIA redist — `pip install torch --index-url https://developer.download.nvidia.com/compute/redist/jp/v60` | Standard pip — `pip install torch torchvision` |
| OpenCV | Ships with JetPack — may conflict with ROS 2's OpenCV | Install normally via apt |
| RViz2 GPU rendering | Broken under X11 on JetPack 6 — falls back to software rendering (`llvmpipe`). Use Foxglove Studio over rosbridge instead | Works normally with any NVIDIA/AMD GPU + standard drivers |
| colcon build time | 1–2 hours (12-core ARM) | 20–40 min (typical desktop CPU) |
| `nmcli` Ouster setup | Required — `eno1` is the physical Ethernet port | Required if running with real hardware — interface name may differ (check with `ip link`) |
| Simulation (no hardware) | AWSIM can run on Jetson but GPU-limited | Recommended platform for AWSIM / CARLA simulation |

### Key build flag differences

**Jetson AGX Orin:**
```bash
colcon build --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-w" \
    -DCUDA_ARCH_BIN="8.7"
```

**x86_64 (example for RTX 3080, compute capability 8.6):**
```bash
colcon build --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-w" \
    -DCUDA_ARCH_BIN="8.6"
```

> Find your GPU's compute capability at:
> https://developer.nvidia.com/cuda-gpus

### Running without hardware (laptop, bag file replay)

On a laptop without the physical gokart or Ouster sensor, you can still run
localization and planning by replaying a pre-recorded bag file instead of launching
the live sensor:

```bash
# Skip Terminals 1 and 2 (no hardware needed)
# Start the vehicle stub and Autoware as normal (Terminals 3 and 4)
# Then replay a bag file in a fifth terminal:
ros2 bag play ~/bags/your_recording/ --clock --rate 0.8
```

The `--clock` flag makes ROS 2 use the bag's timestamps rather than wall clock,
which is required for NDT and the EKF to behave correctly with recorded data.

---

## Documentation

1. [What is Autoware?](docs/autoware_intro.md)
2. [LiDAR Sensing Pipeline](docs/lidar_sensing_pipeline.md)
3. [NDT and EKF](docs/ndt_ekf.md)
4. [Camera Sensing Pipeline](docs/camera_sensing_pipeline.md)

---

## Hardware

| Component | Details |
|---|---|
| Compute | NVIDIA Jetson AGX Orin Developer Kit |
| OS | Ubuntu 22.04, JetPack 6.2.2 |
| LiDAR | Ouster OS0-32, connected via Ethernet |
| LiDAR IP | `169.254.96.62` |
| Jetson interface | `eno1` → `169.254.33.10` (static, persistent via `nmcli`) |

---

## Step 1 — Set up the Jetson network interface for the LiDAR

The Ouster OS0 communicates over a link-local Ethernet connection. Configure a
persistent static IP on the Jetson so the sensor is reachable after every reboot:

```bash
sudo nmcli con add type ethernet \
  con-name ouster-link \
  ifname eno1 \
  ipv4.method manual \
  ipv4.addresses 169.254.33.10/16 \
  connection.autoconnect yes

sudo nmcli con up ouster-link
```

Verify the sensor is reachable:

```bash
ping 169.254.96.62
```

---

## Step 2 — Clone Autoware and install dependencies

### 2.1 Clone the Autoware repository

```bash
git clone https://github.com/ASAS-Labs/Autoware-Buggy.git
cd Autoware-Buggy
```

### 2.2 Install all dependencies via the setup script

Autoware-Buggy provides a setup script that automatically installs everything needed
(ROS 2 Humble, rosdep, CUDA toolchain, colcon, vcstool, etc.) in one step. When
prompted for a BECOME password, enter your Ubuntu desktop password.

```bash
./setup-dev-env.sh
```

> The full list of available release tags is on the
> [Autoware releases page](https://github.com/autowarefoundation/autoware/releases).

### 2.3 Import all package sources

```bash
vcs import src < repositories/autoware.repos
```

> **Jetson / no discrete GPU:** if you are on a Jetson (where CUDA comes bundled
> with JetPack rather than being installed by the playbook) or on a machine without
> an NVIDIA GPU, skip the NVIDIA-specific steps:
> ```bash
> ansible-playbook autoware.dev_env.install_dev_env --skip-tags nvidia
> ```

If the playbook fails on any step, consult the official
[Troubleshooting guide](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/#troubleshooting).

### 2.4 Install ROS package dependencies

```bash
source /opt/ros/humble/setup.bash
sudo apt update && sudo apt upgrade
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

---

## Step 3 — Build Autoware

This step takes 1–2 hours on the Jetson AGX Orin.

```bash
colcon build --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-w" \
    -DCUDA_ARCH_BIN="8.7"
```

> `-DCMAKE_CXX_FLAGS="-w"` suppresses warning spam so real errors are visible.
> `-DCUDA_ARCH_BIN="8.7"` targets the Jetson AGX Orin's Ampere GPU specifically.

If the build fails on **OpenCV ximgproc headers**:
```bash
sudo apt install -y libopencv-contrib-dev
```
Then re-run the `colcon build` command above.

Once the build finishes, source the workspace:

```bash
source ~/Autoware-Buggy/install/setup.bash
echo "source ~/Autoware-Buggy/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Step 4 — Apply gokart changes

Run the install script from the repo root. This copies the gokart sensor kit and
vehicle packages into the right locations inside `src/`, patches the NDT params,
and rebuilds the affected packages.

```bash
chmod +x install.sh
./install.sh
```

### What `install.sh` does

| Action | Detail |
|---|---|
| Copies `gokart_sensor_kit_launch` | From `gokart_packages/` into `src/launcher/autoware_launch/sensor_kit/` |
| Copies `gokart_vehicle_launch` | From `gokart_packages/` into `src/launcher/autoware_launch/vehicle/` |
| Patches NDT params | Lowers `converged_param_nearest_voxel_transformation_likelihood` from `2.3` → `1.5`, raises `num_threads` to `8` |
| Makes scripts executable | `chmod +x Buggyscripts/launch_gokart.sh` |
| Rebuilds modified packages | `colcon build --packages-select gokart_vehicle_description gokart_vehicle_launch gokart_sensor_kit_description gokart_sensor_kit_launch` |

---

## Step 5 — Prepare the map

Download the prebuilt NYU campus map from the Drive link below:

> **Map download:** https://drive.google.com/drive/folders/14vZKf0e51XjsdPlE20VT9tFJ4AI_fajN?usp=drive_link

It contains:
- `pointcloud_map.pcd` — LIO-SAM pointcloud map
- `lanelet2_map.osm` — Lanelet2 vector map

Place the files at:

```bash
mkdir -p ~/Autoware-Buggy/nyu-map
# Copy pointcloud_map.pcd and lanelet2_map.osm into ~/Autoware-Buggy/nyu-map/
```

> To build your own map for a different environment, see
> [docs/lidar_sensing_pipeline.md](docs/lidar_sensing_pipeline.md).

---

## Step 6 — Run the stack

Open four terminals. Start them in order — each step depends on the previous one
being live before continuing.

### Terminal 1 — LiDAR driver

```bash
source ~/Autoware-Buggy/install/setup.bash
ros2 launch ouster_ros sensor.launch.xml \
  sensor_hostname:=169.254.96.62 \
  lidar_mode:=1024x20 \
  point_type:=xyzir \
  "proc_mask:=IMU|PCL" \
  timestamp_mode:=TIME_FROM_ROS_TIME \
  pub_static_tf:=false \
  viz:=false
```

Wait until the terminal shows the sensor connected and pointcloud publishing before
moving on.

### Terminal 2 — Converter + IMU relay

```bash
bash ~/Autoware-Buggy/Buggyscripts/launch_gokart.sh
```

Starts `ouster_to_xyzirc.py` (PointXYZIRC converter + QoS bridge) and `imu_relay.py`
(`/ouster/imu` → `/sensing/imu/imu_data`) as background processes. Ctrl+C kills both.
Wait ~3 seconds for both nodes to initialize.

### Terminal 3 — Vehicle interface stub

```bash
source ~/Autoware-Buggy/install/setup.bash
python3 ~/Autoware-Buggy/Buggyscripts/vehicle_interface_stub.py
```

You should see:
```
[vehicle_interface_stub]: Vehicle stub started -- velocity from NDT PoseStamped delta, steering from IMU
```

This must be running before Autoware starts, otherwise the operation mode transition
manager will block autonomous mode indefinitely.

### Terminal 4 — Autoware

```bash
source ~/Autoware-Buggy/install/setup.bash
ros2 launch autoware_launch autoware.launch.xml \
  vehicle_model:=gokart_vehicle \
  sensor_model:=gokart_sensor_kit \
  map_path:=$HOME/Autoware-Buggy/nyu-map \
  launch_sensing_driver:=false \
  launch_sensing:=true \
  launch_localization:=true \
  launch_perception:=true \
  launch_planning:=true \
  launch_control:=true \
  rviz:=true \
  gnss_enabled:=false \
  "initial_pose:=[0.0, 0.0, 0.0, 0.0, 0.0, 0.707, 0.707]"
```

---

## Step 7 — Verify everything is working

```bash
# NDT localization rate (~10-12 Hz) and confidence score (aim for > 2.3)
ros2 topic hz /localization/pose_estimator/pose
ros2 topic echo /localization/pose_estimator/nearest_voxel_transformation_likelihood --field data

# EKF fused pose — should be 50 Hz
ros2 topic hz /localization/kinematic_state

# Vehicle status from stub — should be 50 Hz
ros2 topic hz /vehicle/status/velocity_status

# Operation mode — mode: 2 = autonomous, is_autonomous_mode_available: true = ready
ros2 topic echo --once /api/operation_mode/state

# Live control commands in human-readable format
ros2 topic echo /control/command/control_cmd | awk \
  '/velocity/{printf "Speed: %.1f mph  ", $2*2.23694} \
   /steering_tire_angle/{printf "Steer: %.1f deg\n", $2*57.2958}'
```

Set a destination by clicking **2D Goal Pose** in RViz2. A trajectory will appear and
control commands will update as Autoware plans and follows the route.

---

## Repository structure

```
Autoware-Buggy/
├── gokart_packages/
│   ├── gokart_sensor_kit_launch/   # Ouster OS0 sensor kit — xacro + calibration yaml
│   └── gokart_vehicle_launch/      # Gokart vehicle description — vehicle_info.param.yaml
├── Buggyscripts/
│   ├── ouster_to_xyzirc.py         # Ouster → PointXYZIRC converter node
│   ├── imu_relay.py                # IMU topic relay
│   ├── vehicle_interface_stub.py   # Vehicle interface stub
│   └── launch_gokart.sh            # Launches converter + IMU relay
├── docs/
│   ├── autoware_intro.md
│   ├── lidar_sensing_pipeline.md
│   ├── ndt_ekf.md
│   └── camera_sensing_pipeline.md
├── repositories/
│   └── autoware.repos              # vcs import source list
├── install.sh                      # Gokart setup script
└── README.md
```

---

## Customization — Where to edit algorithms and config

All config files live under `src/launcher/autoware_launch/autoware_launch/config/`.
Edit the file, then rebuild only the affected package with `colcon build --packages-select <package>` and re-source.

---

### Localization

#### NDT Scan Matcher (LiDAR → map alignment)
```
src/launcher/autoware_launch/autoware_launch/config/localization/ndt_scan_matcher/ndt_scan_matcher.param.yaml
```
| Parameter | What it does | Gokart value |
|---|---|---|
| `trans_epsilon` | Convergence threshold — smaller = stricter | `0.01` |
| `step_size` | Newton line search step | `0.1` |
| `converged_param_nearest_voxel_transformation_likelihood` | Minimum score to accept a match | `1.5` (lowered from `2.3`) |
| `num_threads` | CPU threads for NDT | `8` |

Rebuild after editing:
```bash
colcon build --packages-select autoware_ndt_scan_matcher
source install/setup.bash
```

---

#### EKF Localizer (Kalman filter — sensor fusion)
```
src/launcher/autoware_launch/autoware_launch/config/localization/ekf_localizer.param.yaml
```
| Parameter | What it does |
|---|---|
| `predict_frequency` | EKF prediction rate (Hz) — default 50 |
| `tf_rate` | TF publish rate (Hz) |
| `extend_state_step` | How many steps ahead to predict |
| `pose_smoothing_steps` | Smoothing window for pose input |
| `twist_smoothing_steps` | Smoothing window for twist input |
| `pose_measure_uncertainty_time` | Measurement uncertainty threshold |
| `proc_stddev_vx_c` | Process noise — longitudinal velocity |
| `proc_stddev_wz_c` | Process noise — yaw rate |

> Increase `proc_stddev_vx_c` and `proc_stddev_wz_c` if the EKF is too slow to follow
> sudden motion changes. Decrease them if the pose estimate is noisy.

Rebuild after editing:
```bash
colcon build --packages-select autoware_ekf_localizer
source install/setup.bash
```

---

#### Pointcloud preprocessor (crop box, voxel downsample)
```
src/launcher/autoware_launch/autoware_launch/config/localization/ndt_scan_matcher/pointcloud_preprocessor/crop_box_filter_measurement_range.param.yaml
src/launcher/autoware_launch/autoware_launch/config/localization/ndt_scan_matcher/pointcloud_preprocessor/voxel_grid_filter.param.yaml
```
Edit these to change the LiDAR range window fed into NDT or the voxel leaf size.

---

### Vehicle

#### Gokart dimensions and kinematics
```
gokart_packages/gokart_vehicle_launch/gokart_vehicle_description/config/vehicle_info.param.yaml
```
| Parameter | What it does |
|---|---|
| `wheel_base` | Distance between front and rear axles (m) |
| `wheel_tread` | Track width (m) |
| `front_overhang` / `rear_overhang` | Overhang distances (m) |
| `max_steer_angle` | Maximum steering angle (rad) |

After editing, run `install.sh` again to copy the updated file into `src/` and rebuild:
```bash
./install.sh
```

---

### Control

#### Lateral controller — MPC
```
src/launcher/autoware_launch/autoware_launch/config/control/trajectory_follower/lateral/mpc.param.yaml
```
Key params: `weight_lat_error`, `weight_heading_error`, `weight_steering_input`,
`prediction_horizon`, `prediction_sampling_time`. Tune these if the gokart
oscillates laterally or cuts corners.

#### Longitudinal controller — PID
```
src/launcher/autoware_launch/autoware_launch/config/control/trajectory_follower/longitudinal/pid.param.yaml
```
Key params: `kp`, `ki`, `kd` for speed tracking. Increase `kp` if the gokart
undershoots target speed; reduce if it overshoots and oscillates.

Rebuild control after editing:
```bash
colcon build --packages-select autoware_trajectory_follower_node
source install/setup.bash
```

---

### Sensor kit

#### Ouster OS0 extrinsics (TF calibration)
```
gokart_packages/gokart_sensor_kit_launch/gokart_sensor_kit_description/config/sensor_kit_calibration.yaml
```
Edit the `x`, `y`, `z`, `roll`, `pitch`, `yaw` values to match the physical
mounting position of the Ouster relative to `base_link`. After editing run `./install.sh`.

#### Sensor kit URDF
```
gokart_packages/gokart_sensor_kit_launch/gokart_sensor_kit_description/urdf/sensor_kit.xacro
```
Edit to change the TF tree structure or add new sensors.

---

### Custom nodes (Buggyscripts)

| File | What to edit |
|---|---|
| `Buggyscripts/ouster_to_xyzirc.py` | QoS settings, field mapping, topic names |
| `Buggyscripts/imu_relay.py` | IMU topic remapping, frame ID |
| `Buggyscripts/vehicle_interface_stub.py` | Velocity estimation method, steering source, publish rate |
| `Buggyscripts/launch_gokart.sh` | Which nodes to launch, startup order |

These are plain Python scripts — edit in place and restart the relevant terminal.
No rebuild needed.

---

## Troubleshooting

**Ouster driver can't connect to sensor**
```bash
sudo nmcli con up ouster-link
ip addr show eno1     # should show 169.254.33.10
ping 169.254.96.62
```

**NDT score stays below 1.0 after setting initial pose**

Use the **2D Pose Estimate** button in RViz2 to manually click the vehicle's position
on the map and set its approximate heading. The launch command's `initial_pose` is an
identity quaternion which may not match where the vehicle actually is.

**`is_autonomous_mode_available: false` with all topics publishing**

The vehicle interface stub (Terminal 3) must be started before Autoware (Terminal 4).
Stop Terminal 4, confirm Terminal 3 is running, then relaunch Terminal 4.

**`colcon build` fails on OpenCV ximgproc**
```bash
sudo apt install -y libopencv-contrib-dev
```

**`colcon build` fails on CUDA architecture**

Make sure `-DCUDA_ARCH_BIN="8.7"` is included in the build command — `8.7` is the
Jetson AGX Orin's compute capability. Other Jetson models use different values.
