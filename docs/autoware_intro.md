# What is Autoware, Generally

Autoware is an open-source software stack for autonomous driving, built on ROS 2 and
maintained by the Autoware Foundation (originally developed by Tier IV). It provides
every software layer a self-driving vehicle needs — sensing, localization,
perception, planning, and control — as a single integrated system, rather than as
separate tools someone has to wire together themselves.

## An analogy

Think of Autoware the way you'd think of the human nervous system driving a body:
your eyes and inner ear are **sensing**, your brain figuring out exactly where you
are in a room is **localization**, your brain recognizing the chair and the person
walking toward you is **perception**, your brain deciding "I should walk around the
chair to reach the door" is **planning**, and the nerve signals telling your legs
exactly how to move are **control**. Autoware is that same chain of
sense → locate → perceive → decide → act, implemented as software running on a
vehicle instead of a nervous system running a body.

## It's bigger than it looks

Autoware isn't one program — it's an enormous collection of individually maintained
ROS 2 packages bundled together into one cohesive system. Building it from source
for this project pulled in **471 separate packages**: drivers, filters, planners,
controllers, message definitions, visualization tools, and the glue code connecting
all of them. That scale is exactly why Autoware exists as a project in the first
place — assembling a working autonomy stack from scratch, package by package, would
otherwise take years.

## The subsystems, briefly

- **Sensing** — raw sensor drivers and the preprocessing that turns raw data
  (pointclouds, images, IMU readings) into a clean, correctly-formatted stream the
  rest of the stack can consume.
- **Localization** — answers "where am I?" NDT scan matching against a prebuilt map,
  fused with IMU data via an EKF, to produce a continuous, accurate pose estimate.
- **Perception** — answers "what's around me?" Detecting, classifying, and tracking
  obstacles and dynamic objects from LiDAR and/or camera data.
- **Planning** — answers "where should I go and how?" Mission planning finds a route
  through a map; motion planning turns that route into a concrete, drivable
  trajectory with speed and obstacle-avoidance baked in.
- **Control** — converts the planned trajectory into actuator commands — steering
  angle and target speed — at high frequency.
- **Map** — the static knowledge the rest of the stack relies on: a 3D pointcloud
  map for localization, and a Lanelet2 vector map encoding lanes, rules, and
  connectivity for planning.
- **Vehicle Interface** — the bridge between Autoware's software commands and a
  specific vehicle's actual actuators (or, as on this platform, a software stub
  standing in for that interface).

## Autoware can also drive in simulation — CARLA

Autoware isn't limited to real hardware. The Technical University of Munich (TUM)
maintains a bridge connecting Autoware Universe to **CARLA**, a widely used
open-source driving simulator, letting the exact same Autoware stack drive a
simulated vehicle in a simulated city instead of (or in addition to) real hardware —
useful for testing planning and control logic without needing the physical platform
running.

- **TUMFTM/Carla-Autoware-Bridge** (Carla 0.9.15, Autoware Universe Humble):
  [https://github.com/TUMFTM/Carla-Autoware-Bridge](https://github.com/TUMFTM/Carla-Autoware-Bridge)
