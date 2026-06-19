# LiDAR Sensing Pipeline

This document walks through the LiDAR sensing chain on the gokart platform, node by
node, from the raw Ouster OS0-32 output to a populated obstacle objects topic. Each
node is explained using Situation / Task / Action / Result (STAR), reflecting the
actual debugging and configuration work done to get that node functioning correctly
on this platform — not just what the node does in the abstract.

```
OS0-32 driver
  → ouster_to_xyzirc.py (custom)
  → /sensing/lidar/concatenated/pointcloud
  → crop_box_filter
  → scan_ground_filter (common_ground_filter)
  → occupancy_grid_based_outlier_filter
  → voxel_based_compare_map_filter + voxel_grid_downsample_filter
  → euclidean_cluster
  → shape_estimation
  → obstacle_pointcloud_based_validator
  → multi_object_tracker
  → /perception/object_recognition/objects
```

---

## `ouster_to_xyzirc.py` (custom node)

**Situation:** The Ouster ROS 2 driver does not publish pointclouds in the
`PointXYZIRC` layout Autoware's preprocessing chain (`crop_box_filter`, NDT, etc.)
requires — `x, y, z` as `float32`, `intensity` as `uint8`, `return_type` as `uint8`,
`channel` as `uint16`, packed into a 16-byte point step. It also publishes with
`BEST_EFFORT` QoS, while Autoware's downstream subscribers use `RELIABLE`.

**Task:** Get a correctly-formatted, correctly-QoS'd pointcloud onto a topic Autoware's
sensing pipeline can actually consume.

**Action:** Wrote a Python node that subscribes to the raw Ouster pointcloud,
re-packs each point into the exact `PointXYZIRC` byte layout Autoware expects, and
republishes it with `RELIABLE` QoS.

**Result:** `crop_box_filter` and everything downstream began receiving valid,
correctly-typed data. Without this node the entire perception and localization
pipeline comes up cleanly with no errors but never receives a single point — the
failure mode is silent, which made this the first thing worth verifying whenever the
pipeline appeared "stuck."

---

## `crop_box_filter`

**Situation:** This is the entry point of the obstacle segmentation pipeline,
subscribing to `/sensing/lidar/concatenated/pointcloud`. Like any LiDAR mounted on a
vehicle, the OS0 returns some points from its own housing/mount and from the vehicle
frame immediately around it, which aren't useful obstacle data.

**Task:** Confirm the node is correctly wired into the converted pointcloud stream and
scoped to a sensible region of interest before the heavier ground-filtering and
clustering stages run.

**Action:** Verified via `ros2 node info` that `crop_box_filter` subscribes to
`/sensing/lidar/concatenated/pointcloud` (i.e. the converter's output, not the raw
Ouster topic) and traced its output forward through the chain. Driver-level
`min_range`/`lidar_min_range` was also evaluated as a complementary, sensor-side way to
exclude very-near returns before they even reach this filter.

**Result:** Confirmed as a correctly-connected pass-through stage feeding the ground
filter at the expected rate; no custom tuning beyond Autoware's defaults was found to
be necessary once the converter was producing correctly-scaled data.

---

## `scan_ground_filter` (node: `common_ground_filter`)

**Situation:** Raw, non-ground-filtered LiDAR points include the ground plane itself,
which would otherwise be picked up by the clustering stage as a giant false "obstacle"
spanning the whole field of view.

**Task:** Separate ground points from non-ground (potential obstacle) points before
anything downstream tries to cluster them.

**Action:** Used Autoware's stock ray/scan-based ground filter unmodified. Confirmed
its output topic, `/perception/obstacle_segmentation/single_frame/pointcloud`, via
`ros2 node info` and `ros2 topic hz`.

**Result:** Publishing correctly once fed real data, observed at roughly 6–9 Hz
depending on Jetson CPU load — slightly below the LiDAR's native 20 Hz, which tracks
with this being one of the more CPU-intensive stages on the Jetson AGX Orin.

```bash
ros2 topic hz /perception/obstacle_segmentation/single_frame/pointcloud
```

---

## `occupancy_grid_based_outlier_filter`

**Situation:** After ground removal, sparse single-return points and other noise can
remain that don't correspond to anything coherent. This node also turned out to be the
center of the project's biggest perception bug: its output topic,
`/perception/obstacle_segmentation/pointcloud`, had **two** publishers —
`occupancy_grid_based_outlier_filter` (the real pipeline) and
`vehicle_interface_stub` (which was publishing an empty pointcloud to satisfy
Autoware's perception-topic requirements before a real pipeline existed).

**Task:** Identify why the perception pipeline appeared to detect clusters
(`euclidean_cluster` showed real, populated cluster data with real points) but never
produced final tracked/predicted objects.

**Action:** Used `ros2 topic info -v` on `/perception/obstacle_segmentation/pointcloud`
to list every publisher on the topic, which revealed the vehicle interface stub as a
second, competing publisher racing the real outlier filter output. Removed the stub's
dummy obstacle-pointcloud publisher entirely (see `CHANGES.md` Section 2).

**Result:** With the dummy publisher removed, the real outlier filter became the sole
source on this topic, and the rest of the chain began receiving consistent, correctly
formatted data rather than intermittently empty pointclouds.

---

## `voxel_based_compare_map_filter` + `voxel_grid_downsample_filter`

**Situation:** The filtered pointcloud at this stage is still dense and contains many
points that correspond to static, already-mapped structures (buildings, ground
clutter) rather than dynamic obstacles.

**Task:** Reduce point count for performance and isolate points that represent
dynamic objects not already present in the prebuilt map.

**Action:** Used the default `voxel_based_compare_map_filter`, which compares live
points against the prebuilt PCD map and discards points within `distance_threshold`
(default `0.5` m) of a mapped point, followed by voxel grid downsampling.

```bash
ros2 param get /perception/object_recognition/detection/voxel_based_compare_map_filter distance_threshold
```

**Result:** `/perception/object_recognition/detection/pointcloud_map_filtered/pointcloud`
publishing at roughly 19–20 Hz with dynamic-only points feeding into clustering. This
threshold is the main tuning knob if real obstacles standing close to mapped static
geometry (e.g. a person standing next to a building) are being filtered out along with
the map points — raising it makes the filter more conservative about what it discards.

---

## `euclidean_cluster`

**Situation:** Individual filtered points need to be grouped into discrete object
candidates before anything downstream can reason about "an obstacle" rather than just
"a point."

**Task:** Confirm LiDAR-only obstacle detection is functioning, without relying on any
camera (none is installed on this platform yet).

**Action:** Used Autoware's stock `euclidean_cluster` node unmodified. Verified its
output directly with `ros2 topic echo --once` on the clusters topic rather than
trusting `ros2 topic hz` alone, to confirm the clusters actually contained real point
data and weren't just empty placeholder messages.

```bash
ros2 topic echo --once /perception/object_recognition/detection/clustering/clusters \
  | grep -E "feature|width"
```

**Result:** Confirmed multiple real clusters detected live from the environment (for
example, 14 clusters in one frame, with individual cluster widths of 11 and 23 points
respectively) at roughly 7–9 Hz — proof that LiDAR-only clustering is genuinely
functional on this hardware, independent of whether the rest of the chain was passing
that data through correctly.

---

## `shape_estimation` → `obstacle_pointcloud_based_validator` → `multi_object_tracker`

**Situation:** Clusters with real point data were confirmed (above), but
`/perception/object_recognition/objects` — the topic the planner actually consumes —
was consistently publishing `objects: []` at 10 Hz, i.e. empty despite real upstream
detections.

**Task:** Trace exactly where in the clusters → shape estimation → validation →
tracking chain the real detections were being dropped.

**Action:** Walked the chain stage by stage with `ros2 topic hz` and `ros2 topic echo`
on each intermediate topic. Found and removed the vehicle interface stub's dummy
`PredictedObjects` publisher on `/perception/object_recognition/objects` (the same
class of bug as the pointcloud topic above — see `CHANGES.md` Section 2).
`multi_object_tracker` was confirmed publishing at 10 Hz once the dummy publishers
were removed.

**Result:** Removing the dummy publishers fixed the topic-collision part of the
problem. At last check, final objects on `/perception/object_recognition/objects` were
still empty, and inspecting cluster contents directly showed
`existence_probability: 0.0` and `dimensions: {x: 0, y: 0, z: 0}` in the `shape` field
of individual clusters — pointing at `shape_estimation` failing to fit a bounding box
to otherwise-valid cluster points. **This remains an open item** (tracked in
`CHANGES.md` Section 9) — the next debugging step is to inspect
`shape_estimation`'s bounding-box fitting parameters and confirm it's receiving the
`channel`/`intensity` fields it may depend on from the upstream `PointXYZIRC` data.

---

## Useful commands for this pipeline

```bash
# Check the full chain is alive, stage by stage
ros2 topic hz /sensing/lidar/concatenated/pointcloud
ros2 topic hz /perception/obstacle_segmentation/single_frame/pointcloud
ros2 topic hz /perception/obstacle_segmentation/pointcloud
ros2 topic hz /perception/object_recognition/detection/pointcloud_map_filtered/pointcloud
ros2 topic hz /perception/object_recognition/detection/clustering/clusters
ros2 topic hz /perception/object_recognition/tracking/objects
ros2 topic hz /perception/object_recognition/objects

# Find every publisher on a topic (how the dummy-publisher bug was found)
ros2 topic info /perception/obstacle_segmentation/pointcloud -v

# Inspect actual cluster content, not just rate
ros2 topic echo --once /perception/object_recognition/detection/clustering/clusters
```
