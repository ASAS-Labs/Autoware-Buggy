# NDT and EKF

This document covers Autoware's two core localization components on this platform:
the NDT scan matcher (the "where am I relative to the map right now" estimator) and
the EKF localizer (the fusion layer that turns intermittent NDT poses into a smooth,
continuous state estimate).

---

## NDT Scan Matching — why not ICP?

The classic approach to matching a live LiDAR scan against a prebuilt map is **ICP**
(Iterative Closest Point): for every point in the new scan, find its nearest
neighbor in the map, then solve for the rigid transform that minimizes the sum of
those point-to-point (or point-to-plane) distances, and repeat until convergence.
ICP works, but it has two practical weaknesses that matter for a moving vehicle: it
needs a reasonably good initial guess to converge to the right answer rather than a
local minimum, and its cost surface is not smooth — small changes in which point gets
matched to which neighbor can cause it to jump around.

Autoware's default localization method, `ndt_scan_matcher`, uses **NDT** (Normal
Distributions Transform) instead. Rather than matching individual points, NDT divides
the prebuilt map into a 3D grid of voxels and represents the distribution of points
inside each voxel as a single Gaussian (a mean and covariance). Matching a new scan
then becomes an optimization problem: find the 6-DOF transform (x, y, z, roll, pitch,
yaw) that maximizes the total probability of every scan point under the Gaussian of
whichever voxel it falls into. This produces a smoother cost surface than raw ICP, is
less sensitive to exact point correspondences, and tends to converge more reliably
from a coarser initial guess — which is why Autoware uses it as the default rather
than ICP.

NDT's output includes a convergence score —
`nearest_voxel_transformation_likelihood` — that tells you how good the match is, not
just what the match was. On this platform the threshold for "trust this pose" was
lowered from Autoware's default (`2.3`) to `1.5` (see `CHANGES.md` Section 3), trading
strict gating for more continuous pose availability in lower-feature outdoor areas,
relying on the EKF's covariance weighting to handle lower-confidence poses gracefully
rather than dropping them outright.

```bash
# Convergence score, live
ros2 topic echo /localization/pose_estimator/nearest_voxel_transformation_likelihood --field data

# Raw NDT pose output rate (typically 10-12 Hz)
ros2 topic hz /localization/pose_estimator/pose
```

| Score | Interpretation |
|---|---|
| `> 3.0` | Excellent match |
| `2.3 – 3.0` | Converged (Autoware's stock threshold) |
| `1.5 – 2.3` | Below stock threshold but still usable (this platform's threshold) |
| `< 1.0` | Very poor match — treat the pose as unreliable |

---

## EKF Localizer

NDT only produces a pose every ~80–100 ms (10–12 Hz). That's not fast enough on its
own for smooth trajectory tracking or stable control — the vehicle needs a continuous
position and velocity estimate, not one that updates in discrete jumps. The
**EKF localizer** (`ekf_localizer`, part of Autoware's `pose_twist_fusion_filter`)
solves this by fusing the intermittent NDT pose with IMU data:

- Between NDT updates, the filter **predicts** forward using IMU angular velocity and
  the gyro odometer, advancing the state estimate continuously.
- When a new NDT pose arrives, the filter **corrects** that prediction, with the
  correction weighted by the NDT pose's covariance — a high-confidence NDT pose pulls
  the estimate strongly toward it, a low-confidence one barely moves it.

The result is a continuous, smooth pose-and-velocity estimate published at **50 Hz**,
regardless of NDT's slower update rate.

### Key outputs

- **`/tf`** — the EKF continuously broadcasts the `map → base_link` transform. This
  is what every other node that needs "where is the vehicle right now" actually
  listens to, rather than querying the localizer directly.
- **`/localization/kinematic_state`** (also referred to in Autoware documentation as
  `pose_twist_fusion_filter`'s kinematic state output, and in some Autoware versions
  exposed under the explicit topic name
  `/localization/pose_twist_fusion_filter/kinematic_state`) — a `nav_msgs/Odometry`
  message containing both the fused pose and the fused twist (linear/angular
  velocity), at 50 Hz. This is the canonical "where am I and how fast am I moving"
  topic for the rest of the stack.

```bash
# Fused pose+twist output, 50 Hz expected
ros2 topic hz /localization/kinematic_state

# /tf broadcast rate
ros2 topic hz /tf

# Inspect actual content — pose and twist fields
ros2 topic echo --once /localization/kinematic_state
```

### Why velocity wasn't taken from EKF twist on this platform

It's worth noting `/localization/kinematic_state`'s twist field, while textbook-correct
in principle, was found to be unreliable for driving this particular vehicle's
software stub (see `CHANGES.md` Section 2): the EKF's twist estimate is itself fed by
the gyro odometer, which depends on the vehicle's *reported* velocity — and on this
platform, that reported velocity comes from the vehicle interface stub, which in turn
was being asked to read EKF twist. That circular dependency meant EKF twist stayed
near zero regardless of actual motion. The stub was changed to compute velocity from
raw NDT pose position deltas instead, which has no such dependency on vehicle-reported
state. This is purely a workaround specific to not having a real CAN bus / wheel
encoder feeding genuine vehicle velocity into the gyro odometer — on a platform with
real wheel speed sensors, EKF twist would be the correct source to use directly.

---

## Quick reference — checking localization health end to end

```bash
ros2 topic hz /localization/pose_estimator/pose          # NDT raw pose, ~10-12 Hz
ros2 topic echo /localization/pose_estimator/nearest_voxel_transformation_likelihood --field data
ros2 topic hz /localization/kinematic_state               # EKF fused pose+twist, 50 Hz
ros2 topic hz /tf                                          # map -> base_link broadcast
```
