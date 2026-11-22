# meldog_description

Robot description of **Meldog**, a 12-DoF quadruped: URDF/xacro model, meshes, simulator
layers (Gazebo Fortress, MuJoCo) and the launch files used to visualize and simulate it.

The model is split into layers:

- A **simulator-independent core**: kinematics, dynamics, visual and collision geometry.
- **Layers** that add simulator- or hardware-specific tags on top of the core.

All physical parameters live in YAML files, in SI units. The xacro files contain only structure.

Target stack: ROS 2 Humble, Ignition Gazebo Fortress (DART physics), ros2_control,
`gz_ros2_control`.

---

## Quick start

```bash
# from the workspace root
colcon build --packages-select meldog_description --symlink-install
source install/setup.bash

# RViz: core model + joint_state_publisher_gui sliders
ros2 launch meldog_description meldog_rviz.launch.xml

# Gazebo: robot + ros2_control + all controllers
ros2 launch meldog_description meldog_gazebo.launch.py
```

Both launch files take `collision:=simple` (default) or `collision:=mesh`. See
[Collision models](#collision-models).

Use `--symlink-install`: launch files read the description from the install space. Without
it, every change to a YAML or xacro file needs a rebuild.

---

## Architecture

### Layers

```
                    description/meldog.urdf.xacro     <- single entry point
                     args: sim, collision, controllers_file
                                   |
        +--------------------------+---------------------------+
        |                          |                           |
   sim:=none                  sim:=gazebo                 sim:=mujoco
        |                          |                           |
        v                          v                           v
  +-----------+   +-----------+  +--------------+  +---------------------+
  |   core    | + |   core    |+ | ros2_control |+ | gazebo              |
  +-----------+   +-----------+  | (GazeboSim-  |  | friction, sensors,  |
                                 |  System)     |  | gz plugins          |
                                 +--------------+  +---------------------+
                                                   core (absolute mesh paths)
                                                   + mujoco (<compiler/>)

  core/          <- config/robot/geometry.yaml, inertial.yaml, actuators.yaml
  gazebo/        <- config/gazebo/gazebo.yaml
  ros2_control/  <- actuator limits from the core (actuators.yaml)
```

| Layer | Directory | Contains | Must not contain |
|---|---|---|---|
| Core | [description/core/](description/core/) | links, joints, inertia, visual + collision geometry | `<gazebo>`, `<ros2_control>`, `<mujoco>` tags |
| ros2_control | [description/ros2_control/](description/ros2_control/) | joint and sensor interfaces; the hardware plugin is a macro parameter | simulator-specific tags |
| Gazebo | [description/gazebo/](description/gazebo/) | surface friction, joint springs, IMU and contact sensors, gz plugins | anything the core already defines |
| MuJoCo | [description/mujoco/](description/mujoco/) | `<mujoco><compiler/>` options for MuJoCo's URDF importer | |

The core never depends on a layer. Layers only *reference* the core, by link and joint names
and by the properties loaded from `config/robot/*.yaml`. A new backend means a new layer,
for example a real-robot `ros2_control` system with a different hardware plugin. It is
selected by a new branch in `meldog.urdf.xacro`. The core does not change.

### Generating the URDF by hand

```bash
xacro description/meldog.urdf.xacro                              # = sim:=none collision:=simple
xacro description/meldog.urdf.xacro sim:=none   collision:=mesh
xacro description/meldog.urdf.xacro sim:=gazebo collision:=simple
xacro description/meldog.urdf.xacro sim:=gazebo controllers_file:=/abs/path/my_controllers.yaml
xacro description/meldog.urdf.xacro sim:=mujoco
```

| Argument | Values | Default | Meaning |
|---|---|---|---|
| `sim` | `none`, `gazebo`, `mujoco` | `none` | which layers are added on top of the core |
| `collision` | `simple`, `mesh` | `simple` | collision geometry, see below |
| `controllers_file` | absolute path | `controllers/meldog_joint_controllers.yaml` | controller_manager parameters for `gz_ros2_control` (`sim:=gazebo` only) |

Unknown values of `sim` and `collision` stop xacro with an error.

> **`description/meldog_core.urdf.xacro` is *not* the core.** It is a deprecated wrapper,
> equivalent to `meldog.urdf.xacro sim:=gazebo`, kept only for packages that still include it.
> New code should use `meldog.urdf.xacro`.

---

## Where to find what

```
meldog_description/
├── description/                     xacro sources
│   ├── meldog.urdf.xacro            ENTRY POINT (sim / collision / controllers_file args)
│   ├── meldog_core.urdf.xacro       deprecated wrapper = sim:=gazebo
│   ├── core/
│   │   ├── core.urdf.xacro          loads config/robot/*.yaml, macro meldog_core: trunk, IMU, 4 legs
│   │   ├── leg.urdf.xacro           macro meldog_leg: links H/UL/LL/F, joints T/H/K/F
│   │   └── part.urdf.xacro          macro meldog_part: one link (visual, collision list, mirrored inertial)
│   ├── ros2_control/
│   │   ├── meldog_ros2_control.urdf.xacro   macro meldog_ros2_control_system(name, plugin)
│   │   └── transmissions.urdf.xacro         transmission macros (not used yet)
│   ├── gazebo/
│   │   └── meldog_gazebo.urdf.xacro         macro meldog_gazebo(controllers_file, collision)
│   └── mujoco/
│       └── meldog_mujoco.urdf.xacro         macro meldog_mujoco (not verified in MuJoCo yet)
├── config/
│   ├── robot/                       simulator-independent parameters, SI units
│   │   ├── geometry.yaml            joint offsets, limb lengths, IMU pose, meshes, collision shapes
│   │   ├── inertial.yaml            masses, CoMs, inertia tensors (given for the LF leg)
│   │   └── actuators.yaml           motor torque/speed, gear ratio, joint limits, damping, friction
│   └── gazebo/
│       └── gazebo.yaml              friction coefficients, IMU rate/noise, contact and odometry rates
├── controllers/
│   ├── meldog_joint_controllers.yaml        controller_manager config used by the Gazebo launch
│   └── meldog_trajectory_controllers.yaml   old JointTrajectoryController config (not used)
├── launch/
│   ├── meldog_gazebo.launch.py      Gazebo + robot_state_publisher + spawn + controllers
│   └── meldog_rviz.launch.xml       robot_state_publisher + joint_state_publisher_gui + RViz
├── meshes/
│   ├── visual/                      STL in millimeters (decimated to 4k-12k triangles)
│   └── collision/                   STL in millimeters, used with collision:=mesh
├── rviz/meldog_config.rviz          RViz config
├── worlds/simple.sdf                Gazebo world: ground plane, 1 ms step, Contact + IMU systems
├── cl_commands/single_joint_test.txt  example `ros2 topic pub` commands for joint_controller
└── test/test_description.py         pytest checks of all xacro variants (see Tests)
```

### Which file do I edit?

| I want to change... | File |
|---|---|
| masses, centers of mass, inertia | [config/robot/inertial.yaml](config/robot/inertial.yaml) |
| limb lengths, joint offsets, IMU position, meshes | [config/robot/geometry.yaml](config/robot/geometry.yaml) |
| simplified collision spheres/boxes | [config/robot/geometry.yaml](config/robot/geometry.yaml), `links.<part>.collision.simple` |
| joint limits, damping, Coulomb friction, motor torque/speed, gear ratio | [config/robot/actuators.yaml](config/robot/actuators.yaml) (keep `joint_params` in the controllers YAML in sync) |
| ground friction, IMU noise, sensor rates | [config/gazebo/gazebo.yaml](config/gazebo/gazebo.yaml) |
| controller list, gains, controller rates | [controllers/meldog_joint_controllers.yaml](controllers/meldog_joint_controllers.yaml) and `CONTROLLERS` in [launch/meldog_gazebo.launch.py](launch/meldog_gazebo.launch.py) |
| physics step, world contents | [worlds/simple.sdf](worlds/simple.sdf) |
| spawn pose, Gazebo verbosity | [launch/meldog_gazebo.launch.py](launch/meldog_gazebo.launch.py) |
| link/joint structure | [description/core/](description/core/) |

---

## Robot model

### Names and frames

Frames follow REP-103: x forward, y left, z up. The base link is `trunk_link`.

| Prefix | Leg |
|---|---|
| `LF` | left front |
| `RF` | right front |
| `LR` | left rear |
| `RR` | right rear |

Each leg `<P>` is a chain `trunk_link → <P>H_link → <P>UL_link → <P>LL_link → <P>F_link`:

| Joint | Type | Parent → child | Axis | Positive rotation |
|---|---|---|---|---|
| `<P>T_joint` | revolute | `trunk_link` → `<P>H_link` (hip block) | +x for left legs, −x for right legs | leg moves **outward** (abduction), on all legs |
| `<P>H_joint` | revolute | `<P>H_link` → `<P>UL_link` (upper limb) | −y | leg swings **forward** |
| `<P>K_joint` | revolute | `<P>UL_link` → `<P>LL_link` (lower limb) | −y | lower limb swings **forward** |
| `<P>F_joint` | fixed | `<P>LL_link` → `<P>F_link` (foot) | | |
| `imu_joint` | fixed | `trunk_link` → `imu_link` | | |

- **Zero pose:** all legs point straight down. The foot frames sit at
  (±0.258, ±0.203, −0.500) m in `trunk_link`.
- **Foot frame vs. contact point:** `<P>F_link` is the **center of the foot sphere**. With
  `collision:=simple`, the contact point with flat ground is 0.035 m lower, at the sphere
  radius.
- Total: 18 links, 12 actuated joints, 5 fixed joints.

### Left/right and front/rear mirroring

The YAML files describe **one leg, LF**. The macros mirror the values for the other legs:

- **Right legs** (`sy = -1`): always mirrored in y.
- **Rear legs** (`sx = -1`): mirrored in x only for parts with `mirror_front_rear: true`. Only
  the hip block has it; upper and lower limbs are the same part front and rear.
- **What gets mirrored:**
  - joint origins;
  - CoMs;
  - inertia products: x → −x flips `ixy` and `ixz`, y → −y flips `ixy` and `iyz`;
  - origins of primitive collision shapes.
- **Meshes are not mirrored.** Mirrored parts have their own STL files in CAD, selected per
  leg in `geometry.yaml`.

### Collision models

Selected with `collision:=`. Defined per link in `geometry.yaml` under `collision.simple` and
`collision.mesh`.

| Model | Trunk | Hip block | Upper limb | Lower limb | Foot |
|---|---|---|---|---|---|
| `simple` (default) | box 0.73 × 0.28 × 0.14 m | 2 spheres | 6 spheres | 7 spheres | 1 sphere r = 0.035 m |
| `mesh` | STL | STL | STL | STL | STL |

Use `simple` for simulation and controllers: sphere and box contacts are much cheaper and
more stable in DART than mesh contacts. The trunk box leaves out the handles and the nose. It
is narrower than the real trunk, so it does not intersect the thigh hubs. Use `mesh` when the
exact shape matters, for example self-collision checks or visualizing collisions in RViz.

Rules for collision shapes:

- `<collision>` elements have **no `name` attribute**. With a name, sdformat drops the
  friction of the foot after merging it into the lower limb (see the Gazebo section).
- The **foot must have exactly one collision shape**, because the contact sensor points at
  it. xacro fails if it does not.

### Units and sources

- All parameters are SI: m, rad, kg, kg·m², N·m, rad/s. STL files are in mm and are scaled by
  `mesh_scale: 0.001`.
- Masses and inertias of the legs come from CAD.
- The trunk is a 10 kg uniform cuboid approximation (see the comment in `inertial.yaml`).
- Joint-side limits are computed from the motor and the gear ratio:
  - effort = 3.5 N·m × 9.97 = 34.895 N·m;
  - velocity = 188.5 rad/s / 9.97 = 18.906 rad/s.

---

## Gazebo layer

[launch/meldog_gazebo.launch.py](launch/meldog_gazebo.launch.py) starts the simulation in this
order:

1. `robot_state_publisher` with `xacro meldog.urdf.xacro sim:=gazebo collision:=<arg>`.
2. Gazebo with [worlds/simple.sdf](worlds/simple.sdf) (`-r -v 2`).
3. `ros_gz_bridge` for `/clock`.
4. The robot is spawned from the `robot_description` topic, at z = 0.55 m.
5. After the spawn, a single `spawner` loads and activates the controllers in the order of
   `CONTROLLERS`: `joint_state_broadcaster`, `joint_controller`, `imu_sensor_broadcaster`,
   `contact_sensors_broadcaster`, `joint_forward_trajectory_controller`.
   `joint_forward_trajectory_controller` is chained to `joint_controller`, so it comes last.

`gz_ros2_control` reads the URDF from `robot_state_publisher` and the controller parameters
from `controllers_file`.

### ros2_control interfaces

System `GazeboSimSystem` (plugin `gz_ros2_control/GazeboSimSystem`):

| Name | Interfaces |
|---|---|
| 12 joints `<P>{T,H,K}_joint` | command: `effort` (±34.895 N·m). State: `position`, `velocity`, `effort` |
| sensor `imu_sensor` | `orientation.{x,y,z,w}`, `angular_velocity.{x,y,z}`, `linear_acceleration.{x,y,z}` |
| sensors `LFF_contact`, `RFF_contact`, `LRF_contact`, `RRF_contact` | `contact` (binary, no forces) |

Sensor names must be the same in three places:

- the ros2_control block;
- the Gazebo `<sensor>`;
- the broadcaster parameters in the controllers YAML.

The tests check this.

### Rates

| What | Rate | Set in |
|---|---|---|
| physics step | 1 kHz (1 ms) | `worlds/simple.sdf` |
| controller_manager | 1000 Hz | controllers YAML |
| broadcasters (joint state, IMU, contact) | 500 Hz | controllers YAML; each `update_rate` must divide the CM rate |
| IMU sensor | 1000 Hz | `gazebo.yaml` |
| foot contact sensors | 200 Hz | `gazebo.yaml` |
| base odometry plugin | 200 Hz | `gazebo.yaml` |

### Gazebo-specific details

- **Friction:**
  - feet: μ = 1.2;
  - every other link: μ = 0, so only the feet grip the ground.

  DART reads only `mu`/`mu2` (plus `fdir1`, `slip1/2`, `restitution`). The ODE parameters
  from Gazebo Classic (`kp`, `kd`, `minDepth`, ...) are ignored and are not set.
- **Foot contact sensors:** each foot is attached by a fixed joint, so sdformat merges ("lumps")
  it into the lower limb. Its collision is renamed to
  `<P>LL_link_fixed_joint_lump__<P>F_link_collision_<N>`, where N is the number of the lower
  limb's own collision shapes. The Gazebo macro computes N from `geometry.yaml`: 7 for
  `simple`, 1 for `mesh`. Adding a sphere to the lower limb therefore needs no manual fix.
- **IMU noise** is currently 0. Realistic values are kept as comments in `gazebo.yaml`.
- The `BaseOdometryPublisher` gz plugin and the custom controllers are provided by other
  packages of the workspace:
  - `joint_controller`;
  - `joint_forward_trajectory_controller`;
  - `contact_sensors_broadcaster`.

  They are not declared in `package.xml`.

Example commands for `joint_controller` are in
[cl_commands/single_joint_test.txt](cl_commands/single_joint_test.txt).

---

## Using the model in another package

For a controller, state estimator or planner, generate the **core** URDF: `sim:=none`, with
simple collisions. It has only links and joints, and the collision geometry needs no mesh
files.

```python
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

meldog_urdf = ParameterValue(
    Command(['xacro ',
             PathJoinSubstitution([FindPackageShare('meldog_description'),
                                   'description', 'meldog.urdf.xacro']),
             ' sim:=none collision:=', LaunchConfiguration('collision')]),
    value_type=str)   # required, otherwise launch tries to parse the URDF as YAML

Node(package='my_package', executable='my_node',
     parameters=[{'robot_description': meldog_urdf, 'use_sim_time': True}])
```

In XML launch files, use `<param name="robot_description" type="str" value="$(command 'xacro ...')"/>`.
`type="str"` is required, for the same reason.

When running together with Gazebo, you can subscribe to `/robot_description` instead (QoS
`transient_local`). It is published by `robot_state_publisher` and has the same core. URDF
parsers such as urdfdom, KDL and Pinocchio ignore the extra `<gazebo>` and `<ros2_control>`
tags. This keeps one source of truth.

On ROS 2 Humble, ros2_control controllers do not get the URDF from the controller manager.
A ros2_control controller has to subscribe to `/robot_description` itself, in
`on_configure`, outside the real-time loop.

For a floating-base model, for example Pinocchio
`buildModelFromXML(urdf, JointModelFreeFlyer(), model)`, the root is `trunk_link`.

---

## Tests

```bash
# from the package directory (needs a sourced ROS 2 environment)
python3 -m pytest test/test_description.py -v
# or
colcon test --packages-select meldog_description && colcon test-result --verbose
```

[test/test_description.py](test/test_description.py) runs on the **source tree**. It
generates every `sim` × `collision` variant and checks:

- URDF validity (`check_urdf`), and that every variant can be passed through a ROS 2 XML
  launch file.
- Argument handling:
  - defaults;
  - unknown values are rejected;
  - the legacy wrapper is equal to `sim:=gazebo`.
- The core contains no simulator tags.
- Model contents:
  - the kinematic tree;
  - joint limits against `actuators.yaml`;
  - collision shapes against `geometry.yaml`;
  - mirroring;
  - total mass;
  - physically valid inertia tensors.
- ros2_control effort limits, and `joint_controller` position limits, against the URDF.
- MuJoCo mesh paths are absolute and exist.
- Gazebo SDF:
  - contact sensors point at existing collisions;
  - every collision gets friction;
  - the sensors match ros2_control.

  These tests run only if `ign sdf` or `gz sdf` is available.

Run the tests after every change to the xacro or YAML files.

---

## Pitfalls

- **Comments inside xacro macros end up in the URDF.** A comment containing a colon followed
  by a space breaks YAML parsing of `robot_description` in XML launch files. Put comments
  above macro definitions, or directly in front of a xacro element (xacro drops those).
- **Do not name `<collision>` elements.** See [Collision models](#collision-models).
- **Controller `update_rate` must divide the controller_manager `update_rate`** (1000 Hz).
- **`$(dirname)` does not work** in the installed xacro version (2.1.1). Use
  `$(find meldog_description)` or paths relative to the file with `xacro.load_yaml`.
- **Joint limits are defined twice:** in `actuators.yaml` and in `joint_params` of the
  controllers YAML. The test catches a mismatch.

---

## Known limitations / TODO

- Foot mass and inertia are 0 (no data yet, `inertial.yaml`).
- The IMU position is a placeholder (`geometry.yaml`, `imu.xyz`).
- The trunk inertia is a cuboid approximation, not from CAD.
- MuJoCo layer:
  - generated, but not tested in MuJoCo;
  - missing the trunk free joint, actuators, sensors and contact parameters, to be added
    on the MJCF side.
- There is no real-robot ros2_control system yet. It needs a new branch in
  `meldog.urdf.xacro` that calls `meldog_ros2_control_system` with the hardware plugin.
- `transmissions.urdf.xacro` and `controllers/meldog_trajectory_controllers.yaml` are not used.
- The simulation has only been checked through the generated URDF/SDF and the tests. It has
  not been tuned in a running Gazebo session after the restructuring.
