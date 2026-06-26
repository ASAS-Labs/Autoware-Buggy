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
### 2.2 Install all dependencies via the the setup script 
Autoware-Buggy provides a setup script that automatically installs everything needed (ROS 2 Humble, rosdep, CUDA toolchain, colcon, vcstool, etc.) in one step. When prompted BECOME password, just enter your ubuntu desktop password. 

```bash
./setup-dev-env.sh
```

> The full list of available release tags is on the
> [Autoware releases page](https://github.com/autowarefoundation/autoware/releases).

### 2.3 Import all package sources

Autoware provides an Ansible playbook that automatically installs everything needed
(ROS 2 Humble, rosdep, CUDA toolchain, colcon, vcstool, etc.) in one step. This is
the official recommended approach and is much less error-prone than installing each
dependency manually.

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
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
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
source ~/Autoware-buggy/install/setup.bash
echo "source ~/autoware/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Step 4 — Clone this repository and apply gokart changes

```bash
cd ~
git clone https://github.com/ASAS-Labs/Autoware-Buggy.git
cd Autoware-Buggy
chmod +x install.sh
./install.sh
```

### What `install.sh` does

| Action | Detail |
|---|---|
| Copies `gokart_sensor_kit` | Custom Ouster OS0 sensor kit — `sensor_kit.xacro` + `sensor_kit_calibration.yaml` |
| Copies `gokart_vehicle` | Gokart vehicle description — `vehicle_info.param.yaml` (wheelbase, dimensions) |
| Patches NDT params | Lowers `converged_param_nearest_voxel_transformation_likelihood` from `2.3` → `1.5`, raises `num_threads` to `8` |
| Copies Python nodes | `ouster_to_xyzirc.py`, `imu_relay.py`, `vehicle_interface_stub.py`, `launch_gokart.sh` |
| Rebuilds modified packages | `colcon build --packages-select gokart_vehicle gokart_sensor_kit` |

After `install.sh` completes, the gokart packages are visible inside the Autoware
workspace and the custom nodes are in `~/` and `~/autoware/`.

---

## Step 5 — Prepare the map

The drive link: `https://drive.google.com/drive/folders/14vZKf0e51XjsdPlE20VT9tFJ4AI_fajN?usp=drive_link` contains the prebuilt NYU campus map:
- `pointcloud_map.pcd` — LIO-SAM pointcloud map
- `lanelet2_map.osm` — Lanelet2 vector map

Copy it to the expected location:

```bash
cp -r ~/Autoware-Buggy/map/nyu-map ~/autoware/nyu-map
```

> To build your own map for a different environment, see
> [docs/lidar_sensing_pipeline.md](docs/lidar_sensing_pipeline.md) and the mapping
> section in [CHANGES.md](CHANGES.md).

---

## Step 6 — Run the stack

Open four terminals. Start them in order — each step depends on the previous one
being live before continuing.

### Terminal 1 — LiDAR driver

```bash
source ~/autoware/install/setup.bash
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
~/autoware/launch_gokart.sh
```

Starts `ouster_to_xyzirc.py` (PointXYZIRC converter + QoS bridge) and `imu_relay.py`
(`/ouster/imu` → `/sensing/imu/imu_data`) as background processes. Ctrl+C kills both.
Wait ~3 seconds for both nodes to initialize.

### Terminal 3 — Vehicle interface stub

```bash
source ~/autoware/install/setup.bash
python3 ~/autoware/vehicle_interface_stub.py
```

You should see:
```
[vehicle_interface_stub]: Vehicle stub started -- velocity from NDT PoseStamped delta, steering from IMU
```

This must be running before Autoware starts, otherwise the operation mode transition
manager will block autonomous mode indefinitely.

### Terminal 4 — Autoware

```bash
source ~/autoware/install/setup.bash
ros2 launch autoware_launch autoware.launch.xml \
  vehicle_model:=gokart_vehicle \
  sensor_model:=gokart_sensor_kit \
  map_path:=$HOME/autoware/nyu-map \
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
├── docs/
│   ├── autoware_intro.md
│   ├── lidar_sensing_pipeline.md
│   ├── ndt_ekf.md
│   └── camera_sensing_pipeline.md
├── src/
│   ├── gokart_sensor_kit/       # Ouster OS0 sensor kit — xacro + calibration yaml
│   └── gokart_vehicle/          # Gokart vehicle description — vehicle_info.param.yaml
├── config/
│   └── ndt_scan_matcher.param.yaml
├── scripts/
│   ├── ouster_to_xyzirc.py
│   ├── imu_relay.py
│   ├── vehicle_interface_stub.py
│   └── launch_gokart.sh
├── map/
│   └── nyu-map/
│       ├── pointcloud_map.pcd
│       └── lanelet2_map.osm
├── CHANGES.md                   # Full change log — every modification explained
├── install.sh
└── README.md
```

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
