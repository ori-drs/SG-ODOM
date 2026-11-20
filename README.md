<div align="center">
  <h1>SG-ODOM: Event-Driven Proprioceptive State Estimation with Stability-Gated Contact Updates for Legged Robots</h1>
</div>

SG-ODOM is a GTSAM-based, event-driven proprioceptive odometry framework
for legged robots. Rather than applying a kinematic correction at every
incoming contact measurement, corrections are triggered by contact events:
a validated touchdown immediately triggers a correction, and, in the
absence of a new touchdown, a maximum dead-reckoning interval bounds how
long the estimate relies on IMU propagation alone. Touchdowns are validated
through stability-gated contact classification: a foot is accepted for a
kinematic update only once it has reached a sufficiently stable support
condition (sufficient vertical ground-reaction force with limited force
variation, held for a validation interval), rather than at the earliest
instant of physical touchdown.

The framework separates robot-specific sensing and kinematics (a per-robot
front end that computes forward kinematics and ground-reaction-force-based
contact classification) from a common estimation back end. Two estimator
configurations implement the same event-driven contact-update and contact
interfaces: an invariant EKF (`invariant_ekf`) and a fixed-lag smoother
(`fixed_lag_combined_bias`). See
[Estimator Configurations](#estimator-configurations) for how they differ, 
and [Tested Platforms](#tested-platforms) for the three robots it has been
deployed on (Spot, ANYmal D, and Unitree G1).

The node consumes IMU data, foot contact states, and (depending on the
platform) joint states for leg forward kinematics and ground-reaction-force
estimation. It publishes odometry, path, and foot contact markers for
RViz, and writes CSV/TUM trajectory outputs for later inspection.

## Tested Platforms

SG-ODOM has been deployed and evaluated on **three different legged
platforms**: two quadrupeds and one humanoid, each with a different
sensor suite. The same estimation back end runs on all three. Only the
robot-specific front end (forward kinematics and contact classification)
and the YAML config change between robots.

<div align="center">
  <img alt="SG-ODOM deployed on Boston Dynamics Spot, ANYbotics ANYmal D, and Unitree G1" src="docs/images/platforms.jpg" width="720" />
  <br/>
  <em>Boston Dynamics Spot (left), ANYbotics ANYmal D (center), Unitree G1 humanoid (right).</em>
</div>

| Platform | Type | Data | Contact sources | Config |
|---|---|---|---|---|
| Boston Dynamics Spot | Quadruped | [GaRLILEO](https://garlileo.github.io/GaRLILEO/) dataset | Foot state or joint-torque GRF | `spot_legged_estimator.yaml` |
| ANYbotics ANYmal D | Quadruped | [GrandTour](https://arxiv.org/abs/2602.18164) dataset | Robot contact state or joint-torque GRF | `anymal_legged_estimator.yaml` |
| Unitree G1 | Humanoid | Self-collected sequences | Joint-torque GRF (`tau_est`) | `g1_legged_estimator.yaml` |

## Overview

<div align="center">
  <img alt="Physical versus stable contact over a stance phase" src="docs/images/stability_gate.png" height="260" hspace="6" vspace="6" />
  <img alt="Event-based contact updates for a trot gait" src="docs/images/event_updates.png" height="260" hspace="6" vspace="6" />
</div>

**Left:** a foot is accepted for estimation only once the ground-reaction
force has settled after impact, not at the first instant of physical
touchdown. **Right:** each stable touchdown (▲) triggers an immediate
kinematic correction, and timeout updates (○) keep IMU-only propagation
shorter than Δt<sub>max</sub>.

## Method

### Event-Based Contact Updates

Contact measurements are processed at the incoming measurement rate, but a
kinematic correction is only applied at selected update events:

- A newly validated touchdown immediately triggers a **contact-triggered
  update**.
- In the absence of a new touchdown, the estimator propagates on IMU alone
  until the elapsed time since the previous correction reaches
  `estimator.max_dead_reckoning_seconds`; the next contact measurement then
  triggers a **timeout-triggered update**, even if the contact condition
  hasn't changed.
- Every correction (touchdown- or timeout-triggered) resets the
  dead-reckoning interval, so updates follow the sequence of contact events
  rather than a fixed sampling grid.

This differs from fixed-rate downsampling: every contact measurement is
still evaluated, only the frequency of *corrections* is reduced. See
[Output Topics](#output-topics) for where `max_dead_reckoning_seconds` is
set per robot.

### Stability-Gated Contact Classification

Physical touchdown does not imply a foot is immediately a stable reference
for a kinematic update — a foot can sustain a substantial ground-reaction
force while still settling or bouncing. Each robot-specific adapter
therefore distinguishes *physical* touchdown from *stable* touchdown using
the signed vertical ground-reaction force `F(t)` (reconstructed from joint
efforts) and its short-horizon variation:

- A contact is accepted only once the force is sufficiently loaded and its
  variation has settled, continuously, for a validation interval — the
  per-robot `*_contact_force_on`, `*_contact_max_force_delta`, and
  `*_contact_force_window_size` parameters (see `sg_odom/config/*.yaml`).
- A contact is released as soon as the force drops and stays below the
  `*_contact_force_off` threshold for the configured off-sample count
  (`*_contact_force_delta_off_samples`).

This activation/release logic is stateful and latched: the contact flag
only changes on these transitions, not on every sample. See
[Input Topics](#input-topics) for the exact parameter names per robot
(`spot_contact_*`, `anymal_contact_*`, `g1_contact_*`).

## Estimator Configurations

Both configurations share the same IMU preintegration, kinematic
measurement model, contact interface, and event-based update schedule
described above; they differ only in state representation and correction
mechanism.

- **Invariant EKF** (`invariant_ekf`): augments the floating-base state
  with one fixed foothold slot per foot, propagates mean/covariance at the
  IMU rate using a left-invariant error convention, and applies each
  contact correction as a sequential EKF update.
- **Fixed-lag smoother** (`fixed_lag_combined_bias`): jointly optimizes
  recent event-based base states and explicit per-contact-episode
  landmarks over a fixed-lag window, connected by preintegrated IMU
  factors. A separate IMU bias variable is associated with each event
  state, connected by a random-walk model, allowing the estimated bias to
  evolve over time.

Exactly one variant is selected per run via `estimator.variants`; see
[Output Topics](#output-topics).

## Dependencies

This package was tested on Ubuntu 24.04 with ROS 2 Jazzy.

The main external dependencies are ROS 2 and GTSAM. Using the
[Docker build](#build-docker-recommended) below satisfies both automatically
inside the container; a host-side install is only needed for the
[native build](#build-native-without-docker).

If building natively, install GTSAM by following the official borglab
repository:

- <https://github.com/borglab/gtsam>

This package was tested with a source build of GTSAM from the `develop` branch.
The tested build reports `GTSAM_VERSION_STRING "4.3a1"` and
`GTSAM_VERSION_NUMERIC=40300`; this is a development build, not an official
GTSAM release.

## Build (Docker, recommended)

The included `Dockerfile` / `docker-compose.yaml` build a ROS 2 image with
GTSAM compiled from source (`develop` branch, `GTSAM_BUILD_UNSTABLE=ON`), so
no host-side GTSAM install is needed. From the repository root:

```bash
docker compose build
docker compose run --rm gtsam_legged_estimator
```

This mounts the repository read/write at `/ros2_ws/src/SG-ODOM`
and drops you into a shell at `/ros2_ws`. Build it there:

```bash
colcon build
source install/setup.bash
```

The [Run](#run) and [Input/Output Topics](#input-topics) sections below apply
unchanged inside the container; substitute `/ros2_ws` for `~/ros2_ws`.

Override the ROS distribution or pin a specific GTSAM commit via env vars
before building the image:

```bash
ROS_DISTRO=humble GTSAM_GIT_REF=<commit-sha> docker compose build
```

## Build (native, without Docker)

Create a ROS 2 workspace, clone this repository into its `src/` directory, and
build from the workspace root:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <this-repository-url> SG-ODOM
cd ~/ros2_ws
colcon build
source install/setup.bash
```

The repository contains several ROS 2 packages, which `colcon build`
discovers and builds in dependency order:

```text
SG-ODOM/
├── sg_odom/                    estimator node (package gtsam_legged_replay_example)
│   ├── config/  launch/  urdf/
│   └── include/  src/
└── msgs/
    ├── legged_estimator_msgs/  this project's diagnostic messages
    ├── any_msgs/               vendored (ANYmal)
    ├── anymal_msgs/            vendored (ANYmal)
    ├── spot_msgs/              vendored (Spot)
    └── unitree_hg/             vendored (Unitree G1)
```

URDF paths in the configs (`*_urdf_path`) are resolved relative to the
installed package share directory unless given as absolute paths.

This requires a source build of GTSAM on the host first (see
[Dependencies](#dependencies)). If GTSAM is installed into a standard CMake
search prefix such as `/usr` or `/usr/local`, no extra path setting is
usually needed. If GTSAM is installed into a custom prefix, expose it before
running `colcon build`:

```bash
export CMAKE_PREFIX_PATH=/path/to/gtsam/install:$CMAKE_PREFIX_PATH
```

Alternatively, point CMake directly at the GTSAM package config directory:

```bash
export GTSAM_DIR=/path/to/gtsam/install/lib/cmake/GTSAM
```

## Run

The node replays recorded ROS 2 bags. Any bag that provides the topics
listed in [Input Topics](#input-topics) for a supported robot works. See
[Tested Platforms](#tested-platforms) for the three robots and datasets the
framework was evaluated on.

In one terminal, replay a recorded ROS 2 bag:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 bag play /path/to/bag_sequence
```

In another terminal, launch the estimator with RViz:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch gtsam_legged_replay_example sg_odom.launch.xml
```

Select the robot-specific config from launch with `robot_type`:

```bash
ros2 launch gtsam_legged_replay_example sg_odom.launch.xml robot_type:=spot
ros2 launch gtsam_legged_replay_example sg_odom.launch.xml robot_type:=anymal
ros2 launch gtsam_legged_replay_example sg_odom.launch.xml robot_type:=g1
```

## Try It: Demo Bags

Two ready-to-play ROS 2 bags (MCAP, zstd-compressed) are provided so you can
run SG-ODOM without collecting your own data. Each one contains only the
topics the estimator needs for that robot. Both are available in this
[Google Drive folder](https://drive.google.com/drive/folders/1XiqZdiJb121IZ3AitXiL4m0ekJZGgcMz),
or download them individually below.

| Robot | Download | Size | Duration | Topics |
|---|---|---|---|---|
| ANYmal D | [sg_odom_anymal_demo.zip](https://drive.google.com/file/d/1PSziQv2mBJOji7fGTb_fBfEc03az6QwX/view?usp=sharing) | 35 MB | 100 s | `/anymal/imu`, `/anymal/state_estimator/anymal_state` |
| Unitree G1 | [sg_odom_g1_demo.zip](https://drive.google.com/file/d/149AfJ_pDDzFnvA3P9gDHhxslGtOo14j6/view?usp=sharing) | 16 MB | 35 s | `/lowstate` |

Download and unzip them into a `bags/` folder at the repository root (it is
git-ignored). With Docker this folder shows up inside the container at
`/ros2_ws/src/SG-ODOM/bags`:

```bash
mkdir -p bags && cd bags
unzip sg_odom_anymal_demo.zip
unzip sg_odom_g1_demo.zip
cd ..
```

Start the container and build the workspace once (see
[Build (Docker, recommended)](#build-docker-recommended)), then open two
shells in it:

```bash
docker compose up -d
docker compose exec gtsam_legged_estimator bash   # run once per terminal
```

**ANYmal D.** Terminal 1 starts the estimator, terminal 2 plays the bag:

```bash
# terminal 1
source install/setup.bash
ros2 launch gtsam_legged_replay_example sg_odom.launch.xml robot_type:=anymal

# terminal 2
source install/setup.bash
ros2 bag play src/SG-ODOM/bags/sg_odom_anymal_demo
```

**Unitree G1.** Same steps, with the G1 config and bag:

```bash
# terminal 1
source install/setup.bash
ros2 launch gtsam_legged_replay_example sg_odom.launch.xml robot_type:=g1

# terminal 2
source install/setup.bash
ros2 bag play src/SG-ODOM/bags/sg_odom_g1_demo
```

Start the estimator before starting playback. The estimate is published on
`/legged_estimator/{odom,path,foot_contacts}`, and trajectories are written
to `./outputs/live` (see [Output Topics](#output-topics)). Add
`use_rviz:=true` to the launch command to open RViz; this needs a display
that the container can reach.

## Input Topics

Robot-specific defaults are defined in `sg_odom/config/spot_legged_estimator.yaml`,
`sg_odom/config/anymal_legged_estimator.yaml`, and `sg_odom/config/g1_legged_estimator.yaml`.

```text
Spot:
  /imu                sensor_msgs/msg/Imu
  /spot/status/feet  spot_msgs/msg/FootStateArray
  /joint_states      sensor_msgs/msg/JointState

Spot supports the same two contact methods. `state` uses
`/spot/status/feet`; `joint_torque_grf` reads the named `front_left_*`,
`front_right_*`, `rear_left_*`, and `rear_right_*` joints from
`/joint_states`, reconstructs each foot GRF, and applies the stateful contact
classifier. The Spot config defaults to `joint_torque_grf`.

ANYmal:
  /anymal/imu                             sensor_msgs/msg/Imu
  /anymal/state_estimator/anymal_state   anymal_msgs/msg/AnymalState

The ANYmal adapter supports exactly two contact methods. `state` uses the
robot-provided `contacts[].state`. `joint_torque_grf` computes the foot position
and Jacobian from the HAA/HFE/KFE joint positions, reconstructs GRF from
`tau = J^T * force` with a damped solve, and applies the stateful vertical-GRF
contact classifier. Select either method at launch with:

```bash
ros2 launch gtsam_legged_replay_example sg_odom.launch.xml \
  robot_type:=anymal anymal_contact_classifier:=state
```

Unitree G1:

  /lowstate                              unitree_hg/msg/LowState

The G1 adapter reads the IMU and the first 12 lower-body motor states from the
same `LowState` packet. Since `unitree_hg/msg/LowState` does not expose foot
force or contact flags, it infers contact from the G1 leg Jacobian and
`tau_est` via `tau = J^T * wrench`. The vertical wrench is filtered
and classified with separate contact-on/contact-off thresholds. The thresholds,
damping, optional FK height guard, and motor indices are configurable in
`sg_odom/config/g1_legged_estimator.yaml`. A dedicated pressure/contact message should
replace this model-based estimate when one is available on the robot.


When `topics.read_joint_states: true`, the node uses `/joint_states` to compute
Spot foot positions with forward kinematics. If this is disabled or the joint
state is too old, the node falls back to the body point contained in the foot
state message when configured to do so.

## Output Topics

Exactly one estimator variant is allowed per live run. Output topics are fixed
so the same RViz configuration works even when the selected variant changes.

```text
/legged_estimator/odom           nav_msgs/msg/Odometry
/legged_estimator/path           nav_msgs/msg/Path
/legged_estimator/foot_contacts  visualization_msgs/msg/MarkerArray
/legged_estimator/g1_contact_debug  std_msgs/msg/Float64MultiArray (G1 only)
```

The G1 contact debug array contains, in order:

```text
[left_raw_N, right_raw_N,
 left_filtered_N, right_filtered_N,
 left_threshold_N, right_threshold_N,
 left_contact, right_contact]
```

The contact entries are `1.0` when active and `0.0` when inactive. The active
threshold switches between `g1_contact_force_on` and
`g1_contact_force_off` to show the detector hysteresis.

If `estimator.variants` contains zero or multiple variants, the node exits at
startup with an error.

```yaml
estimator:
  variants: [invariant_ekf]
```

Supported variant names:

```text
invariant_ekf
fixed_lag_combined_bias
```

## Output Files

The node writes outputs under `estimator.output_dir`, which defaults to
`./outputs/live`.

```text
<variant>_trajectory.csv
<variant>_trajectory_imu.tum
<variant>_metrics.csv
```

The `outputs/` directory is ignored by Git because these files are generated
runtime artifacts.

## Third-Party Message Definitions

`msgs/anymal_msgs/`, `msgs/any_msgs/`, `msgs/spot_msgs/`, and
`msgs/unitree_hg/` are vendored from other open-source projects, not
authored by this project. Only the minimal subset of messages needed by the
estimator is included. Each folder contains the upstream `LICENSE` and a
`README.md` with the pinned source commit and any changes from upstream.

| Package | Messages | Source | License |
|---|---|---|---|
| `anymal_msgs` | `AnymalState`, `Contact` | [leggedrobotics/holistic_fusion](https://github.com/leggedrobotics/holistic_fusion/tree/main/ros/holistic_fusion_anymal_msgs/msg) | BSD-3-Clause |
| `any_msgs` | `ExtendedJointState` | [leggedrobotics/holistic_fusion](https://github.com/leggedrobotics/holistic_fusion/tree/main/ros/holistic_fusion_anymal_msgs/msg) | BSD-3-Clause |
| `spot_msgs` | `FootState`, `FootStateArray` | [rai-opensource/spot_ros2](https://github.com/rai-opensource/spot_ros2/tree/main/spot_msgs/msg) | MIT |
| `unitree_hg` | `LowState`, `IMUState`, `MotorState` | [unitreerobotics/unitree_ros2](https://github.com/unitreerobotics/unitree_ros2/tree/master/cyclonedds_ws/src/unitree/unitree_hg) | BSD-3-Clause |

The ANYmal messages were ported from a ROS 1 package; upstream ships all
three in `holistic_fusion_anymal_msgs`, and here `ExtendedJointState` is
split into `any_msgs` so the type names match ROS 2 ANYmal recordings.
The vendored `spot_msgs/FootStateArray` adds a `std_msgs/Header` that the
upstream definition does not have; see `msgs/spot_msgs/README.md`.

## License

This project is licensed under the BSD-3-Clause License, following GTSAM's
license terms. See the repository root `LICENSE` for details. The
third-party message definitions listed above are excluded from this license;
see [Third-Party Message Definitions](#third-party-message-definitions).
