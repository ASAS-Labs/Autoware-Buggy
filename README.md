# Autoware Universe — Gokart Platform Modifications

This document describes everything that was changed, added, or configured on top of a
stock Autoware Universe source installation to run the stack on the S-AM gokart
platform: a Jetson AGX Orin paired with a single Ouster OS0-32 LiDAR, no GNSS, no
camera-based perception, and a custom (non-OEM) vehicle interface.

Autoware was built from source on JetPack 6.2.2 (471 packages). Everything below is
additional to that base build.

---

## 1. Hardware / Sensor Setup

**Sensor:** Ouster OS0-32, connected via Ethernet, static IP `169.254.96.62`.
Jetson `eno1` interface set to `169.254.33.10` via a persistent `nmcli` connection
profile named `ouster-link` so the link survives reboots without manual reconfiguration.

**Driver launch (standalone, outside Autoware's sensing launch):**
```bash
ros2 launch ouster_ros sensor.launch.xml \
  sensor_hostname:=169.254.96.62 \
  lidar_mode:=1024x20 \
  point_type:=xyzir \
  "proc_mask:=IMU|PCL" \
  timestamp_mode:=TIME_FROM_ROS_TIME \
  pub_static_tf:=false \
  viz:=false
```
`pub_static_tf:=false` is intentional — Autoware's own `sensor_kit_launch` publishes
the TF chain, so the Ouster driver's internal static TF publisher is disabled to avoid
duplicate/conflicting transforms.

**Convenience launcher** — `~/autoware/launch_gokart.sh` starts the OS0 driver, the
IMU relay, and the point cloud converter (below) as background processes from a single
command, with a trap on Ctrl+C to cleanly kill all three.

---

## 2. Custom ROS 2 Nodes

### `ouster_to_xyzirc.py`
Autoware's preprocessing pipeline (crop box filter, NDT, etc.) expects pointclouds in
`PointXYZIRC` layout: `x,y,z` as `float32`, `intensity` as `uint8`, `return_type` as
`uint8`, `channel` as `uint16` (16-byte point step). The Ouster ROS 2 driver does not
publish this layout natively. This node subscribes to the raw Ouster pointcloud and
republishes a byte-for-byte repacked `PointCloud2` matching Autoware's expected field
layout and offsets.

**QoS bridging:** the Ouster driver publishes `BEST_EFFORT`, but Autoware's
`crop_box_filter` and downstream nodes subscribe `RELIABLE`. The converter's output
publisher uses `RELIABLE` QoS to match, otherwise messages are silently dropped at the
subscription boundary.

### `imu_relay.py`
Relays `/ouster/imu` to the IMU topic Autoware expects (`/sensing/imu/imu_data`),
since Autoware's IMU corrector and gyro odometer are wired to a specific topic name
that doesn't match the Ouster driver's default.

### `vehicle_interface_stub.py`
No CAN bus / real vehicle interface exists yet, so this node satisfies Autoware's
vehicle interface contract entirely in software. It publishes, at 50 Hz:
`/vehicle/status/velocity_status`, `/vehicle/status/steering_status`,
`/vehicle/status/gear_status`, `/vehicle/status/control_mode`.

This went through several iterations before reaching the current approach:

| Attempt | Velocity source | Problem |
|---|---|---|
| 1 | EKF position delta | NDT noise on the EKF pose caused false non-zero velocity when stationary → vehicle appeared to drift in RViz after stopping |
| 2 | EKF twist directly | `gyro_odometer` (which feeds EKF twist) itself depends on reported vehicle velocity → circular dependency, twist stayed ~0 |
| 3 (current) | **NDT pose (`PoseStamped`) position delta** | Works — no circular dependency, since NDT pose is independent of vehicle status feedback |

**Current implementation details:**
- Subscribes to `/localization/pose_estimator/pose` (confirmed via `ros2 topic info -v`
  to be `geometry_msgs/msg/PoseStamped`, **not** `PoseWithCovarianceStamped` — the
  topic has two possible types and subscribing to the wrong one silently receives
  nothing).
- `v = sqrt(dx² + dy²) / dt` between consecutive NDT poses.
- Steering estimated from IMU yaw rate via the bicycle kinematic model:
  `δ = atan2(yaw_rate × WHEELBASE, v)`.
- Low-pass filtering (`VEL_ALPHA = 0.4`, `STEER_ALPHA = 0.3`) to smooth NDT jitter.
- `MIN_VEL_THRESH = 0.8 m/s` — below this, velocity/steering/yaw-rate are snapped to
  zero after `STOPPED_FRAMES = 5` consecutive low readings, and the position history is
  reset (`prev_x/y/t = None`) so the next motion starts from a clean delta instead of
  an accumulated stale one. The threshold was raised in stages (0.15 → 0.45 → 0.8) to
  satisfy `operation_mode_transition_manager`'s velocity-matching check for autonomous
  engagement.
- `MAX_DT = 0.3s` and `MAX_VEL = 10.0 m/s` sanity bounds reject stale timestamps and
  NDT jump artifacts respectively.
- **No dummy perception topics are published.** Earlier versions published empty
  `PredictedObjects` and an empty `/perception/obstacle_segmentation/pointcloud` to
  satisfy the planner — this turned out to *override* the real LiDAR perception
  pipeline (clusters were detected correctly downstream but never reached
  `/perception/object_recognition/objects` because the stub's empty data was racing
  the real ground-filter output on the same topic). The dummy perception publishers
  were removed entirely; the real `crop_box_filter → ground_filter →
  occupancy_grid_outlier_filter → euclidean_cluster → shape_estimation → validator →
  multi_object_tracker` chain now runs end-to-end on the Ouster pointcloud.

### `autoware_dashboard.py`
Terminal dashboard subscribing to `/control/command/control_cmd`
(`autoware_control_msgs/msg/Control`), printing speed (km/h), acceleration, and
steering angle/direction with an ASCII steering bar, refreshed live.

---

## 3. Localization Parameter Changes

File: `ndt_scan_matcher.param.yaml` (located via
`find ~/autoware/src -name "ndt_scan_matcher.param.yaml"`).

| Parameter | Stock | Changed to | Reason |
|---|---|---|---|
| `converged_param_nearest_voxel_transformation_likelihood` | 2.3 | 1.5 | Default threshold caused NDT to stop publishing poses in lower-feature outdoor areas; EKF covariance weighting handles low-confidence poses, so a lower threshold trades strict gating for continuous pose availability |
| `num_threads` | 4 | 8 | Use all available Jetson AGX Orin cores for scan matching |
| `skipping_publish_num` | 5 | 1 | Stock value could skip up to 5 consecutive rejected scans before publishing, producing multi-hundred-ms gaps in pose output that the EKF interpreted as motion |

**Recommended bounds:** don't drop the convergence threshold below ~1.0 — that
starts admitting genuinely bad matches rather than just relaxing the gate.

---

## 4. Sensor / TF Tree Configuration

Sensor kit description (`*_sensor_kit_launch` / URDF) updated for the actual physical
chain on the gokart, since the stock Autoware sample sensor kit assumes a different
sensor layout:

```
base_link → sensor_kit_base_link → os_sensor → os_lidar
                                              → os_imu
```

Every node in Autoware stamps messages with a `frame_id`; a single broken link in this
chain causes silent TF lookup failures throughout localization and perception, so this
was verified end-to-end with `ros2 run tf2_tools view_frames` after every sensor kit
change.

`map_path` is loaded with `projection_type: local` (no MGRS/UTM, no GNSS) — the PCD and
Lanelet2 map both use the local frame produced by LIO-SAM, so Autoware's localization
initialization and the Lanelet2 map projector are both set to `Local` rather than the
default geodetic projection.

---

## 5. Mapping Pipeline (Offline, Map-Building Side)

Not part of the live Autoware launch, but the map artifacts it depends on are produced
by this pipeline — documented here since it's a required prerequisite, not a stock
Autoware capability.

**LIO-SAM** (ROS 2 branch, `TixiaoShan/LIO-SAM -b ros2`) is the primary mapper, chosen
for loop closure support on the circuit-style campus route. Required build/config
changes vs. stock LIO-SAM:
- GTSAM rebuilt from source with `-DGTSAM_USE_SYSTEM_EIGEN=ON` — the prebuilt GTSAM
  was compiled against a different Eigen version than ROS 2 Humble ships, causing a
  static assertion failure at compile time.
- `params.yaml`: `baselinkFrame: "os_sensor"` (the recorded bags only contain
  `os_sensor → os_lidar/os_imu`, no `base_link`, so LIO-SAM is told `os_sensor` is the
  root) and `robot.urdf.xacro` rewritten so `os_sensor` is the URDF root with
  `os_lidar`/`os_imu` as fixed children — the stock xacro defines unrelated frame
  names (`lidar_link`, `imu_link`, etc.) that don't exist in the recorded TF tree.
- `imuPreintegration.cpp` failure-detection thresholds raised
  (`vel.norm() > 30 → 100`, `ba.norm()/bg.norm() > 1.0 → 10.0`) after repeated
  "Large velocity, reset IMU-preintegration!" resets on longer (5-mile) recordings;
  requires a rebuild of the `lio_sam` package after editing.
- `point_type:=original` (or `xyzirdt`) used when recording bags intended for
  mapping — `xyzir` lacks per-point timestamps, which disables LIO-SAM's deskew step
  and visibly doubles thin features (tree branches, poles) in the resulting map.
- Final map saved via the `/lio_sam/save_map` service
  (`ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap ...`, using
  `savePCDFileBinary` internally) rather than relying on the Ctrl+C shutdown hook —
  interrupting LIO-SAM mid-write with `savePCDFileASCII` produced a corrupted PCD
  header (`WIDTH`/`POINTS` not matching actual line count).

**FAST-LIO2** (`OmerMersin/FAST_LIO_GPU` fork, chosen over the official
`hku-mars/FAST_LIO` ROS 2 branch because the official branch has a hard Livox
dependency) was evaluated as a faster, GPU-accelerated alternative — useful for quick
iteration but lacks loop closure, so it accumulates drift on longer routes and was not
used for the final campus map.

**Lanelet2 vector map** built using Tier IV's web-based Vector Map Builder
(`tools.tier4.jp/vector_map_builder_ll2`) on top of the exported LIO-SAM PCD, with
`ProjectorType: Local` to match Autoware's `projection_type: local` localization
config.

---

## 6. Perception

No camera is present on this platform, so perception is LiDAR-only, using Autoware's
existing euclidean clustering pipeline (no custom detection nodes were written):

```
/sensing/lidar/concatenated/pointcloud
  → crop_box_filter
  → scan_ground_filter → /perception/obstacle_segmentation/single_frame/pointcloud
  → occupancy_grid_based_outlier_filter → /perception/obstacle_segmentation/pointcloud
  → voxel_grid_downsample_filter / compare_map_filter
  → euclidean_cluster
  → shape_estimation
  → obstacle_pointcloud_based_validator
  → multi_object_tracker
  → /perception/object_recognition/objects
```

The only change required to make this stock pipeline work was removing the vehicle
interface stub's dummy perception publishers (see §2) — the pipeline itself is
unmodified Autoware. `voxel_based_compare_map_filter`'s `distance_threshold` (default
0.5 m) is the main tuning knob if dynamic objects close to mapped static geometry are
being filtered out along with the map points.

---

## 7. Planning / Control

No source changes — stock Autoware mission planning (Lanelet2 graph search), motion
planning (velocity smoother, obstacle stop planner), and control (pure
pursuit/MPC lateral, PID longitudinal) are used as-is. The only adaptation is on the
input side: the vehicle interface stub (§2) supplies the velocity/steering feedback
these modules require, since there is no physical vehicle interface yet.

---

## 8. How to Run the Stack

The stack must be brought up in a specific order because each layer depends on topics
published by the one before it. Running them out of order (e.g. launching Autoware
before the sensor is publishing) causes nodes to wait indefinitely or fail silently on
missing TF/topics.

### Step 1 — LiDAR driver

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

**Why first:** every other node in the stack — the converter, the IMU relay, NDT, the
whole perception pipeline — ultimately consumes data that originates from the sensor.
Nothing downstream can do anything useful until `/ouster/points` and `/ouster/imu` are
actually publishing. `pub_static_tf:=false` is set here because Autoware's own sensor
kit launch (step 4) owns the TF tree — if both the driver and Autoware publish the same
static transforms, you get duplicate/conflicting frames.

### Step 2 — `launch_gokart.sh` (converter + IMU relay)

```bash
~/autoware/launch_gokart.sh
```

This single script starts `imu_relay.py` and `ouster_to_xyzirc.py` as background
processes (with a trap so Ctrl+C kills both cleanly). Equivalent to running, in two
separate terminals:

```bash
python3 ~/imu_relay.py
python3 ~/ouster_to_xyzirc.py
```

**Why this step exists:** Autoware does not understand the Ouster driver's native
output directly.

- `ouster_to_xyzirc.py` re-packs the raw Ouster pointcloud into the exact
  `PointXYZIRC` byte layout Autoware's `crop_box_filter`/NDT pipeline requires, and
  republishes it with `RELIABLE` QoS to match what those nodes subscribe with (the
  Ouster driver itself publishes `BEST_EFFORT`, which Autoware's `RELIABLE`
  subscribers silently ignore).
- `imu_relay.py` republishes `/ouster/imu` onto the topic name
  (`/sensing/imu/imu_data`) that Autoware's IMU corrector and gyro odometer are
  hardwired to subscribe to.

Without this step, Autoware's sensing/localization nodes come up but never receive
any data — there's no error, they just sit idle waiting on topics nothing is
publishing.

### Step 3 — Vehicle interface stub

```bash
python3 ~/autoware/vehicle_interface_stub.py
```

**Why this step exists:** Autoware's planning and control stack will not transition
into autonomous mode — and several nodes (`operation_mode_transition_manager`, the
behavior planner, AEB) will not function correctly — unless something is publishing
the vehicle status topics a real vehicle interface would normally provide:
`/vehicle/status/velocity_status`, `/vehicle/status/steering_status`,
`/vehicle/status/gear_status`, `/vehicle/status/control_mode`. There is no CAN bus
connection on this platform yet, so this node synthesizes those topics from NDT pose
and IMU data instead (see Section 2 for why it computes velocity the way it does).
This must be running before Autoware is launched, or the operation mode transition
manager will report `is_autonomous_mode_available: false` indefinitely with no clear
error pointing at the actual cause.

### Step 4 — Autoware

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

**Why launched last:** Autoware's localization, perception, planning, and control
nodes all depend on topics from steps 1 through 3 being live and correctly formatted.
Bringing Autoware up first just means its nodes spin up cleanly but sit waiting.
`launch_sensing_driver:=false` tells Autoware explicitly not to try to start its own
copy of the Ouster driver, since step 1 already owns that. `gnss_enabled:=false` and
the explicit `initial_pose` are required because this platform has no GNSS — Autoware
needs to be told where it's starting from rather than initializing from a GPS fix.

### Summary

| Step | What | Why it has to come first |
|---|---|---|
| 1 | OS0 driver | Source of all raw sensor data |
| 2 | `launch_gokart.sh` | Converts/relays raw sensor data into the exact topics and formats Autoware expects |
| 3 | `vehicle_interface_stub.py` | Supplies the vehicle status feedback Autoware's mode manager and control stack require to operate |
| 4 | Autoware | Consumes everything above — localization, perception, planning, control |

---

## 9. Known Limitations / Open Items

- **RViz2 GPU acceleration is unresolved on this JetPack 6.2.2 + X11 configuration.**
  `glxinfo` reports `llvmpipe` (software rendering) regardless of `LD_LIBRARY_PATH`,
  `__GLX_VENDOR_LIBRARY_NAME`, or custom `glx_vendor.d` ICD overrides pointing at the
  Tegra `libGLX_nvidia.so.0` — these load but the X server raises `BadValue`/`BadAccess`
  on `glXMakeCurrent`. Root cause appears to be the Tegra X11 driver's GLX path being
  fundamentally limited under JetPack 6 (it primarily targets EGL/Wayland). Workaround
  used for demos: run `rosbridge_server` on the Jetson and visualize via Foxglove
  Studio on a separate laptop, freeing the Jetson's CPU/GPU entirely for Autoware.
- **No real vehicle actuation.** The vehicle interface stub provides velocity/steering
  feedback only — it does not send commands to any motor controller. Planning and
  control compute valid steering/speed commands (`/control/command/control_cmd`), but
  nothing currently consumes them to drive the physical gokart. This is the next
  integration milestone (CAN bus to the motor/steering controllers).
- **PyTorch on the Jetson is CPU-only** (the apt-installed build). A JetPack
  6-specific CUDA wheel is needed before any GPU-accelerated perception model
  (e.g. YOLO, CenterPoint) can run with hardware acceleration.
