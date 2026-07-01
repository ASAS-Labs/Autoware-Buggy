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
