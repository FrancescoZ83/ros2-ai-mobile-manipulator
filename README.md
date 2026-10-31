# Semantic Mobile Manipulation (AMR stack)

A mobile manipulator built from scratch in **ROS 2 Humble** and **Gazebo Fortress**: a differential-drive base, a **UR5e** arm and a custom two-finger gripper. The long-term goal is a robot that understands a semantic request ("bring me the box") and carries it out with navigation, motion planning and perception.

The project is a learning exercise: every file is written by hand, and each step has a success test with measured results (see below).

## Status

| Step | Content | State |
|---|---|---|
| 0 | Development environment (Docker / WSL, ROS 2 Humble, Gazebo Fortress) | done |
| 1 | Robot description (xacro): base, UR5e, gripper | done |
| 2 | Simulation in Gazebo: world, differential drive, ros2_control, bridge | done |
| 3 | Effort control of the arm and the gripper, adaptive payload, driving checks | done |
| 4 | Sensors in Gazebo (LiDAR, RGB-D) | next |
| 5 | Grasp scenarios and world | planned |
| 6 | Nav2 + slam_toolbox | planned |
| 7 | MoveIt 2 (inverse kinematics, planning, collisions) | planned |
| 8 | Perception (PyTorch / ONNX) | planned |
| 9 | Task logic with BehaviorTree.CPP | planned |

## Stack

- ROS 2 Humble, ros2_control, KDL (Orocos)
- Gazebo Fortress (Ignition), `ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control`
- Universal Robots UR5e description (with local modifications for joint damping and friction)
- Docker on WSL

## Repository layout

```
amr_manipulator_description/   URDF/xacro (base, UR5e, gripper, test payload), controller YAML
amr_manipulator_gazebo/        launch file, bridge.yaml, worlds, test models (box, pedestal)
amr_manipulator_control/       custom ros2_control controllers (arm and gripper)
scripts/                       diagnostic and test tools (Python)
```

## Quick start

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# Simulation with arm and gripper controllers
ros2 launch amr_manipulator_gazebo sim.launch.py
# Optional test payload at the TCP [kg]
ros2 launch amr_manipulator_gazebo sim.launch.py payload_mass:=3.0
```

Useful tools (all in `scripts/`):

```bash
# Move the arm to a joint configuration (rad) in T seconds
python3 scripts/arm_goto.py --goal 0.0 -1.5708 1.5708 -1.5708 -1.5708 1.5708 --time 3
# Open / close the gripper (optional grip force [N])
python3 scripts/grip.py close --force 25
python3 scripts/grip.py open
# Where is the TCP and how do its axes point
python3 scripts/tcp_pose.py --frame base_footprint
# Drive straight and report drift, arm error and gripper state
python3 scripts/drive_test.py --speed 0.3 --time 6
```

Note: new files need `colcon build` even with `--symlink-install`; the controller YAML is read only at startup.

## Step 0-2: description and simulation

- **Description.** The robot is one xacro. Everything that exists only in simulation (Gazebo plugins, test payload) sits behind the `use_sim` flag, so the same description can be used on hardware later. A `payload_mass` argument adds a point mass at the TCP for controller tests.
- **Simulation.** The base uses Gazebo's `DiffDrive` (`/cmd_vel`). The arm and the gripper are driven through `gz_ros2_control` in **effort** mode, not position or velocity. All Gazebo and ROS topics are bridged from one file, `config/bridge.yaml`. The launch file starts Gazebo, `robot_state_publisher`, spawns the robot, starts the bridge and then activates `joint_state_broadcaster`, the arm controller and the gripper controller in order.

## Step 3: effort control

No stock controller (PID, `JointTrajectoryController`) is used: the arm controller computes joint torques from the robot's dynamic model.

### Arm: `ArmModelController`

- **Control law.** Computed torque with KDL: `tau = M(q) a + C(q, qd) qd + g(q)`, with `a = qdd_d + 2w*e_dot + w^2*e` and a quintic reference between the current and the target posture. The duration is stretched if needed to respect the joint velocity limits.
- **Three modes in the same controller:** PD with gravity compensation, computed torque, and computed torque with adaptive payload (parameter `mode`).
- **Realism layer (simulation).** Joint friction (Coulomb + viscous) is compensated by feedforward on the reference velocity. Velocity measurements get Gaussian noise (sigma 0.02 rad/s, assumed, not calibrated) and a 20 Hz first-order filter. The effective Coulomb friction of wrist3 in the simulator (0.41 N m, nominal 0.3) was identified from a trace.
- **Adaptive payload (Slotine-Li).** Only the mass at the end effector is adapted. The payload dynamics are linear in the mass, so a second KDL chain with massless links and a 1 kg point mass gives the regressor. The estimate is limited to a range and frozen when the torque saturates. Parameters (gains, scales, noise, filter, estimate) can be changed at runtime with `ros2 param set`.

### Gripper: `GripperController`

- **Coordinates.** Common mode `xc = (qL + qR) / 2` (closing) and differential mode `xd = (qL - qR) / 2` (centring). Both fingers always move the same way towards the centre. A virtual spring on `xd` (max 10 N) keeps the object centred.
- **State machine.** `OPEN` (default, pushed with -5 N against the open stop so it stays open in any orientation), `OPENING`, `CLOSING` (0.05 m/s), `GRASP` (force ramp to 25 N), `EMPTY` (closed on nothing).
- **Contact detection** by stall: the common speed stays below a fraction of the approach speed while the controller pushes with more than 1.5 N.
- **Commands.** `~/command` takes `[mode, force]`. A new `close` while holding only changes the force, with no release. State is published on `~/state`.
- **The simulator limits finger speed to 0.2 m/s.**

## Results

### Arm with a 3 kg payload (RMS tracking error, rad)

| | lift | elbow | wrist1 |
|---|---|---|---|
| Computed torque, payload unknown | 0.0066 | 0.0245 | 0.0247 |
| Adaptive, from the third move | 0.0003 | 0.0005 | 0.0005 |
| No payload (reference) | 0.0002 | 0.0006 | 0.0017 |

The estimate settles at 3.0 kg with an accuracy of about 0.05 kg. At rest it stays at 3.011-3.018 kg over 30 s (no drift). With no payload it stays below 0.095 kg.

### Gripper and real grasp (1 kg box, 60 mm)

- Contact detected at 0.32 s, grip force 25.08 / 24.92 N, box unmoved (< 1 micrometre).
- Lifted box: height 0.75735 m (predicted 0.758), tilt about 0.3 degrees, finger centring error 0.
- Payload estimate with the real box: **1.01 kg** (average of the up and down moves; each direction has a bias of about 0.07 kg caused by arm friction). An estimate below 0.15 kg means "no payload".
- With the estimate near 1 kg the arm under load is as accurate as the unloaded arm.

### Driving straight (2 m, with arm posed, ground truth from Gazebo)

| | Empty gripper | Box held |
|---|---|---|
| Distance (planned 1.98 m) | 1.973 m | 1.971 m |
| Lateral deviation | -1 mm | -1 mm |
| Yaw change | -0.010 deg | -0.013 deg |
| Gripper | open | `GRASP`, 24.98 / 25.02 N, centring error 0.06 mm |

## Known limits

- The lift pose of the arm (tool pointing down, vertical motion) was computed offline with the full DH kinematics (checked against the measured box position to 2 mm). There is **no inverse kinematics at runtime** yet: joint references are given by hand. Planning and IK are the job of MoveIt 2.
- Joint-space interpolation is not a straight line in Cartesian space. Over the short lift the deviation is at most 4.7 mm.
- Stiction at rest is not compensated: elbow, wrist1 and wrist2 stop up to about 3 mrad from the target, wrist3 up to about 0.5-1 degree.
- The payload is a point mass at the TCP: inertia and centre of mass of a real object are not estimated. The Lyapunov guarantee of the adaptive law holds strictly only for equal gains on all joints; here it was checked experimentally.
- Noise and friction parameters are simulation assumptions, not identified on real hardware (except the wrist3 Coulomb value).
- Driving is open loop; straight motion comes from the symmetric simulated robot. A real base needs an outer loop (odometry or LiDAR, for example through Nav2).
- Finger friction uses the simulator default; it is enough to hold 1 kg at 25 N.

### Step 4: sensors

- **2D LiDAR** (`gpu_lidar`): 360 rays over 360 degrees, 10 Hz, range 0.12-12 m, Gaussian noise
  (sigma 1 cm, assumed). Published as `/scan` (`sensor_msgs/LaserScan`, frame `lidar_link`).
- **RGB-D camera** (`rgbd_camera`) on the gripper, in a D435-sized housing: 320x240, 10 Hz,
  69 degrees horizontal field of view, clip 0.1-10 m. Published as `/camera/image`,
  `/camera/depth_image` and `/camera/camera_info`, all in the optical frame
  `camera_color_optical_frame`. The point cloud exists in Gazebo but is not bridged (cost).
- Rendering runs on the CPU (llvmpipe) and is cheap at this resolution: Real Time Factor 0.99
  with both sensors.
- **Verified:**
  - LiDAR range to a pedestal at a known distance: 1.114 m against 1.13 m expected (1 cm noise);
  - the scan is not mirrored: an object on the left appears at +90 degrees;
  - `lidar_link` in TF at (0.34, 0, 0.275);
  - camera and TF end to end: the top of a known box is located within 2 mm (pixel, depth and
    TF are consistent).
- Tools in `scripts/`: `scan_probe.py`, `cam_probe.py`, `cam_check.py`; view in
  `config/sensors_view.rviz`.

Known limits:
- The LiDAR sees the UR5e base (a short sector behind the sensor); it will be masked for SLAM and Nav2.
- Gazebo depth is perfect (no noise, no holes). It is kept as ground truth. Realistic depth will
  come from a stereo pair in the same housing (50 mm baseline) at the perception step.
- An object held between the fingers is closer than the 0.1 m near clip, so it is not visible.
- `camera_info` counts the image corner as the pixel origin (principal point 160, 120): pixel
  indices need a +0.5 shift in projection code.

## Step 5: warehouse world

A small order-picking warehouse, 12 x 8 m, generated from a single description (`config/warehouse_layout.yaml`). The same description also produces the semantic map, so the world and what the robot knows about it cannot drift apart.

- **Building.** Three zones (receiving, storage, packing) separated by two partitions with four 1.2 m doors.
- **Furniture.** Seven shelf units (two rows, three levels at 0.30 / 0.60 / 0.90 m, accessible from both sides), two work benches, two bins on the packing bench, two columns. Level 1 is the top of a solid plinth, so the LiDAR plane (z = 0.275 m) sees every shelf as a solid rectangle.
- **Articles.** Six classes (`cereal_box`, `juice_carton`, `soup_can`, `water_bottle`, `parcel`, `jam_jar`), all primitives with generated textures. **No markers**: a neural network will have to recognise them. Sizes respect the gripper (85 mm) and the level spacing.
- **Stock.** 210 slots (5 per face and level). Each slot is filled with probability 0.30 (seed 1): 69 articles, 28 of them on the faces the robot works from (`pickable`). Articles are static by default; a dynamic copy is only needed when one is picked.
- **Semantic map** (`config/semantic_map.yaml`): zones, doors with the zones they connect, shelves, benches, bins and every slot with its pose and article.

```bash
python3 scripts/make_articles.py --spec amr_manipulator_gazebo/config/articles.yaml \
  --out amr_manipulator_gazebo/models
python3 scripts/generate_world.py \
  --layout amr_manipulator_gazebo/config/warehouse_layout.yaml \
  --template amr_manipulator_gazebo/worlds/empty_world.sdf \
  --articles amr_manipulator_gazebo/config/articles.yaml --static-articles \
  --semantic-out amr_manipulator_gazebo/config/semantic_map.yaml \
  --out amr_manipulator_gazebo/worlds/warehouse.sdf
colcon build --symlink-install --packages-select amr_manipulator_gazebo
ros2 launch amr_manipulator_gazebo sim.launch.py world:=warehouse.sdf [gui:=false] [x:= y:= yaw:=]
python3 scripts/world_check.py --map amr_manipulator_gazebo/config/semantic_map.yaml
```

### Results

| Check | Result |
|---|---|
| Scan from the origin, front / left / right | 4.105 / 0.887 / 0.880 m (expected 4.110 / 0.900 / 0.900; the closest of about 20 noisy rays is 1-2 cm short) |
| Scan from the south aisle (`yaw:=1.5708`) | 0.942 / 2.942 / 1.597 m (expected 0.960 / 2.937 / 1.608) |
| Drive through door P3, 0.3 m/s | 4.673 m (planned 4.68), lateral -1 mm, yaw -0.010 deg |
| Articles at rest in their slots | 28 of 28 within 0.1 mm of the pose in the map |
| Textures | rendered in the Gazebo GUI and in the RGB-D camera |

Real time factor (CPU rendering, 30 s average):

| Scene | GUI | no GUI |
|---|---|---|
| Building and furniture only | 0.83 | - |
| 28 dynamic articles | 0.40 | 0.62 |
| All articles static | 0.73 | **0.98** |

The cost is physics, not rendering: shadows made no difference (0.38 without them). Articles are therefore static in the normal world.

### Known limits (step 5)

- Rendering is on the CPU (llvmpipe); the GPU is left to the neural network.
- Articles are static: a dynamic copy still has to be swapped in when the robot picks one.
- Textures are applied per face (stretched on boxes, a partial crop on the caps of cylinders).
- Shelf levels 1 and 2 leave 0.27 m of free height: a top-down grasp is tight there, level 3 is open above.
- The arm starts at the same pose as `home90`: the gripper is 0.64 m in front of `base_footprint` at 0.63 m, i.e. beyond the base. Driving into a bench at 0.3 m/s ended with the gripper hitting the bench top. The Nav2 footprint and a travel pose are needed before navigation.

## Roadmap

3. **Step 6:** SLAM (slam_toolbox) and navigation (Nav2).
4. **Step 7:** MoveIt 2 with a `FollowJointTrajectory` interface on top of the arm controller.
5. **Step 8:** object perception (PyTorch / ONNX).
6. **Step 9:** task logic with BehaviorTree.CPP.
