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
| 4 | Sensors in Gazebo (LiDAR, RGB-D) | done |
| 5 | Warehouse world and semantic map | done |
| 6 | slam_toolbox + Nav2 | done: travel pose, mapping, map check, navigation through the doors, goals taken from the semantic map |
| 7 | MoveIt 2 (inverse kinematics, planning, collisions) | in progress: trajectory interface, MoveIt configuration, planning scene, reach analysis and the station design done; the gripper sequence is next |
| 8 | Perception (PyTorch / ONNX) | in progress: stereo pair and depth, observation poses, dataset and recognition network done (mean IoU 0.994 on the test split, 0.960 on 36 images of the robot's camera); pose of the target and the grasp next |
| 9 | Task logic with BehaviorTree.CPP | planned |

## Stack

- ROS 2 Humble, ros2_control, KDL (Orocos)
- Gazebo Fortress (Ignition), `ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control`
- Universal Robots UR5e description (with local modifications for joint damping and friction)
- Docker on WSL

## Repository layout

```
amr_manipulator_description/   URDF/xacro (base, UR5e, gripper, test payload), controller YAML
amr_manipulator_gazebo/        launch files (simulation, SLAM), bridge.yaml, worlds, models, saved maps
amr_manipulator_control/       custom ros2_control controllers (arm and gripper), FollowJointTrajectory server
amr_manipulator_moveit_config/ MoveIt 2 configuration (SRDF, kinematics, planners, controller), planning scene
amr_manipulator_perception/    perception nodes: stereo depth now, recognition and pose later
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
# Warehouse world, no GUI. The arm is folded into the travel pose after start-up;
# stow:=false keeps the start-up pose (home90) for manipulation tests
ros2 launch amr_manipulator_gazebo sim.launch.py world:=warehouse.sdf gui:=false [stow:=false]

# Mapping (second terminal), then drive with the keyboard and save the map
ros2 launch amr_manipulator_gazebo slam.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run nav2_map_server map_saver_cli -f amr_manipulator_gazebo/maps/warehouse

# Navigation with the saved map (stop SLAM first: both would publish map -> odom)
ros2 launch amr_manipulator_gazebo nav.launch.py
python3 scripts/nav_goto.py --goal 2.0 0.0 0.0           # x y yaw in the map frame
python3 scripts/nav_goto.py --suite doors --repeat 3     # through the four doors, 3 times
# Places by name, from the semantic map (shelf units, benches, bins, doors, zones)
python3 scripts/nav_semantic.py S3
python3 scripts/nav_semantic.py --tour all --repeat 3
```

Stop a simulation with Ctrl+C and check that no `ign gazebo` process is left: a second Gazebo server makes the controller spawners fail with "Controller already loaded".

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
# Footprint of the platform, of the arm and of the whole robot (needs the simulation running)
python3 scripts/robot_geometry.py
# Rays of the LiDAR that hit the robot itself (needs the simulation running)
python3 scripts/scan_mask.py
# Compare the saved map with the boxes of the SDF world (no ROS needed)
python3 scripts/map_check.py [--tol-m 0.10]
# Turn in place in 90 degree steps: odometry yaw against Gazebo's ground truth (Nav2 stopped)
python3 scripts/turn_test.py [--rate -0.5]
# Pose and clearance of every place of the semantic map (no ROS needed)
python3 scripts/semantic_goals.py [S3 packing_table] [--gap 0.25] [--along]
# MoveIt (move_group running): kinematics, joint and pose goals, straight lines, collisions at any base pose
python3 scripts/moveit_check.py ik | joint --named home90 | pose --dz -0.10 | cartesian --dz 0.10
python3 scripts/moveit_check.py valid --whole --base 0.6 -0.335 0 --named home90
# Reach analysis of the shelf slots from the pick stations (move_group running, no simulation needed)
python3 scripts/reach_check.py --generic --grasps front --orientations facing --gaps 0.20 0.25 0.30
# Stereo pair (simulation started with stereo:=true): put the camera in front of a slot, then measure the stereo depth
python3 scripts/moveit_check.py look --slot S3.L2.n4 [--dist 0.30]
python3 scripts/stereo_check.py --frames 5 --save /tmp/sc
# Labels of the segmentation camera (simulation with stereo:=true dataset:=true): classes seen and overlay
python3 scripts/label_probe.py --save /tmp/lp
# A multi-point trajectory through the FollowJointTrajectory action, with the tracking error
python3 scripts/traj_test.py --goal 0 -1.5708 1.5708 -1.5708 -1.5708 1.5708 --time 6 [--shape via]
# A trajectory that starts where the arm is, not at its last goal, must run (gravity compensation switched off to hold the arm away)
python3 scripts/traj_recovery_test.py
```

MoveIt, with the simulation running (the planning scene of the warehouse is loaded by the launch):

```bash
ros2 launch amr_manipulator_moveit_config move_group.launch.py static_map_odom:=true [rviz:=true]
```

`static_map_odom:=true` publishes an identity `map` to `odom` transform: use it only without AMCL or SLAM (the robot starts at the origin, where the frames coincide).

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
- The LiDAR sees the UR5e base (a short sector behind the sensor); measured and filtered by range in step 6.
- Gazebo depth is perfect (no noise, no holes). It is kept as ground truth. Realistic depth comes from a stereo
  pair in the same housing (50 mm baseline), see step 8.
- An object held between the fingers is closer than the 0.1 m near clip, so it is not visible.
- `camera_info` counts the image corner as the pixel origin (principal point 160, 120): pixel
  indices need a +0.5 shift in projection code.

## Step 5: warehouse world

A small order-picking warehouse, 12 x 8 m, generated from a single description (`config/warehouse_layout.yaml`). The same description also produces the semantic map, so the world and what the robot knows about it cannot drift apart.

- **Building.** Three zones (receiving, storage, packing) separated by two partitions with four 1.2 m doors.
- **Furniture.** Seven shelf units (two rows, three levels at 0.30 / 0.60 / 0.90 m, accessible from both sides), two work benches, two bins on the packing bench, two columns. Level 1 is the top of a solid plinth, so the LiDAR plane (z = 0.275 m) sees every shelf as a solid rectangle.
- **Articles.** Six classes (`cereal_box`, `juice_carton`, `soup_can`, `water_bottle`, `parcel`, `jam_jar`), all primitives with generated textures. **No markers**: a neural network will have to recognise them. Sizes respect the gripper (85 mm) and the level spacing. `make_articles.py` checks them: a box turned by the yaw jitter of the stock (15 degrees) must still be narrower than the opening along the aisle (the fingers close along it when an article is taken from the front), and no article is lower than 0.07 m, because the body of the gripper is 0.06 m thick (see step 7). The parcel was first 0.10 x 0.07 x 0.04 m; it is now 0.06 x 0.08 x 0.09 m.
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
- The arm starts at the same pose as `home90`: the gripper is 0.64 m in front of `base_footprint` at 0.63 m, i.e. beyond the base. Driving into a bench at 0.3 m/s ended with the gripper hitting the bench top. The Nav2 footprint and a travel pose are needed before navigation (travel pose: see step 6).

## Step 6: SLAM and navigation

Footprint and travel pose, LiDAR self-echo, mapping with slam_toolbox, numerical map check, localization (AMCL), navigation (Nav2) through the four doors, and goals taken from the semantic map.

### Footprint and travel pose

Measured with `scripts/robot_geometry.py` (convex hull of the collision geometry in `base_footprint`, with the TF of the running robot).

- **Platform:** x -0.400 .. +0.400, y -0.315 .. +0.315, an 8-vertex polygon: `[[-0.4,-0.25], [-0.1,-0.315], [0.1,-0.315], [0.4,-0.25], [0.4,0.25], [0.1,0.315], [-0.1,0.315], [-0.4,0.25]]`. The wheel axis is at x = 0, so the robot turns on the spot inside a circle of 0.472 m radius; the inscribed radius is 0.315 m. A circular footprint would waste the margin of the 1.2 m doors, so Nav2 gets the polygon.
- **`home90` is not a driving pose:** the TCP is at (0.642, 0.134, 0.628) and the robot reaches x = +0.697.
- **Travel pose** `[pi, -1.5708, 1.5708, -1.5708, -1.5708, 1.5708]`: `home90` with only the base joint turned by 180 degrees, so the arm folds over the chassis. The hull of the whole robot is then **identical** to the platform hull. The arm and the tools span x -0.381 .. +0.370, y -0.210 .. +0.075; the smallest margin is 19 mm, at the rear.
- **Stow at launch.** Once all controllers are active, `sim.launch.py` runs `scripts/arm_goto.py` towards the travel pose (argument `stow`, default `true`). The script is found relative to the launch file, so this needs `colcon build --symlink-install`. The arm first ramps to the start-up pose and then sweeps 180 degrees (about 13 s in total): start in free space, or use `stow:=false`.

| Check | Result |
|---|---|
| Tracking error to the travel pose | max 0.0089 rad (wrist3), other joints <= 0.0031 rad |
| Peak torque | 27.2 N m (lift), 24.9 N m (elbow) |
| Hull of the whole robot | equal to the platform hull |

### LiDAR self-echo

The base of the UR5e crosses the LiDAR plane about 0.19 m behind the sensor. `scripts/scan_mask.py` takes the median of 20 scans with the robot in the middle of the aisle (everything closer than 0.5 m is the robot itself):

- **28 rays** in two symmetric sectors, -171.0 .. -157.9 and +157.9 .. +171.0 degrees, at 0.120 .. 0.153 m. Nothing else is closer than 0.5 m.
- Between the two sectors (about +-9 degrees around 180) the sensor looks past the base: the closest return there is 4.774 m, the receiving bench seen through the west door (4.79 m expected). Probable cause: the base surface is closer than the 0.12 m minimum range and is not rendered; this was not checked in the simulator.

So there is no angular mask: a minimum range of **0.2 m** removes every self-echo (4.7 cm above the farthest one, against a noise of 1 cm) and keeps the valid rays of the window. It is `min_laser_range` in slam_toolbox and AMCL, and `obstacle_min_range` / `raytrace_min_range` in the Nav2 costmaps.

### Mapping

`launch/slam.launch.py` starts slam_toolbox (online, asynchronous, mapping mode) with `config/slam_toolbox.yaml`: the file installed with slam_toolbox, with three changes marked `CHANGED` (`min_laser_range` 0.2, `max_laser_range` 12, `map_update_interval` 2). The robot starts at the world origin, so the frame `map` coincides with the world frame. The simulation is a separate launch, so SLAM can be restarted alone. The map was driven by hand (keyboard teleoperation) through the aisles and the doors and saved in `amr_manipulator_gazebo/maps/warehouse.{yaml,pgm}` (242 x 163 cells, 0.05 m).

### Map check

`scripts/map_check.py` rasterizes the boxes of the SDF that cross the LiDAR plane (walls, partitions, shelf plinths, bench bodies, columns; articles and bench tops are higher) and compares them with the saved map, in both directions:

- every occupied map cell: signed distance to the nearest box (negative = inside, positive = on the free side);
- every exposed face of the world (next to free space reachable from the start): is there an occupied cell within one cell?

| Check | Result |
|---|---|
| Signed distance of the occupied cells | median +0.5 cm, 5th..95th percentile -7.0 .. +6.5 cm, max +9.5 cm |
| Occupied cells within 0.10 m of a box | 100 % (68.9 % within 0.05 m) |
| Exposed faces covered (+-1 cell) | 99.7 %: walls, shelves and benches 100 %, `packing_table` 98.8 %, `column_1` 85.7 % |

The tolerance of 0.10 m (two cells) was chosen after seeing these numbers: the first proposal, 0.05 m, would fail (68.9 %). The error is a band of 2 cells around each face with no systematic offset, and nothing is more than 9.5 cm away, so there is no double wall. Whether the band is normal blur of the occupied cells or a small registration error is not separated by this check.

### Navigation (AMCL + Nav2)

`launch/nav.launch.py` starts Nav2's own bringup (map server, AMCL, planner, controller, behaviors, BT navigator, velocity smoother) with `config/nav2_params.yaml`: the file installed with Nav2, every difference marked `CHANGED`. The simulation is a separate launch. AMCL takes its initial pose (0, 0, 0) from the parameters, because the robot always starts at the world origin.

Main choices:

- **Frames and footprint.** `base_footprint` everywhere (the example uses `base_link`); the 8-vertex polygon of the platform instead of `robot_radius`.
- **Self-echo.** Minimum range 0.2 m in AMCL (`laser_min_range`) and in both costmaps; without it AMCL would use the 28 rays that hit the arm base.
- **Local controller: DWB**, up to 0.4 m/s and 1 rad/s, with the `ObstacleFootprint` critic (the whole polygon on the costmap, not only the robot centre). Goal tolerance 0.10 m and 0.10 rad. Reversing is allowed at the same 0.4 m/s (`min_vel_x`, and the same limit in the velocity smoother).
- **Local costmap** with an obstacle layer instead of the voxel layer: the data is a 2D scan.

`scripts/nav_goto.py --suite doors` sends six goals, through each of the four doors (P1 and P2 into and out of the receiving zone, P3 on the way from receiving to packing, P4 from packing to the north aisle), and compares the final pose with Gazebo's ground truth. Three runs in a row, 18 goals, reversing allowed:

| Goal | mean / max time [s] (simulation) | recoveries | reached |
|---|---|---|---|
| P1 into receiving | 25.9 / 30.9 (see below) | 0 | 3/3 |
| receiving south | 15.2 / 15.8 | 0 | 3/3 |
| P2 into south aisle | 16.2 / 16.4 | 0 | 3/3 |
| P2 out to receiving | 18.6 / 19.7 | 0 | 3/3 |
| receiving to packing (P1, P3) | 28.2 / 29.6 | 1 | 3/3 |
| P4 into north aisle | 21.5 / 22.6 | 0 | 3/3 |

| Over the 18 goals | mean / max / std |
|---|---|
| Final position error against ground truth [m] | 0.085 / 0.169 / 0.032 |
| Final yaw error [deg] | 2.1 / 8.4 / 1.8 |
| Nav2's own pose estimate against ground truth [m] | 0.059 / 0.138 / 0.038 |

The first goal of the first run starts at the origin (4.2 m path, 17.6 s); in the next two runs it starts from the north aisle where the previous run ended (about 10 m, 29-31 s): that is the spread of its row, not noise.

**Reversing.** In the first attempt, without reversing, the run reached 6 goals of 6 but two of them stalled for more than 30 s: "P2 out to receiving" took 66.2 s with 5 recoveries and "P4 into north aisle" 70.1 s with 4. With reversing at 0.2 m/s the same two took 32.6 s and 32.3 s with no recovery, and at 0.4 m/s about 19 s and 22 s (table above). These were single runs; the cause of the original stalls was not isolated. Both goals need the robot to turn around, and the station at (3.8, 0) is too tight for that: the smallest distance between the footprint and an obstacle during a full turn is 0.179 m, against 0.315 m of inscribed radius (the south aisle goal has 0.529 m). Open places such as (4.2, 2.0) have 0.654 m.

**Odometry while turning.** `scripts/turn_test.py` turns in place in four 90 degree steps and compares the odometry yaw with the ground truth:

| Speed | Odometry / truth, counter-clockwise | clockwise |
|---|---|---|
| 0.5 rad/s | 0.936 | 0.934 |
| 1.0 rad/s | 0.997 | 0.993 |

The odometry always reports about 89.4 degrees for a 90 degree step; the body turns 95.6 degrees at 0.5 rad/s and about 90 at 1 rad/s. The centre of the robot moves less than 9 mm, so there is no lateral slip. The error depends on the speed, not on the direction, so one `wheel_separation` cannot fix it (it would be 0.545 m at 0.5 rad/s and 0.578 m at 1 rad/s, against 0.58). The probable cause is the contact of the 0.05 m wide wheel cylinders, but it was not checked and the model was not changed: changing the wheel collision would also change the footprint measured in step 6 and the results of steps 3 and 5. AMCL absorbs the error while the robot drives.

### Goals from the semantic map

`scripts/semantic_goals.py` turns a name of `config/semantic_map.yaml` into a pose, and `scripts/nav_semantic.py` drives there with Nav2 (`nav_goto.py` does the driving and the report).

| Name | Pose |
|---|---|
| `S1`..`S4`, `N1`..`N3` | in the aisle in front of the pick face, bow towards the shelf (yaw -90 degrees on the south row, +90 on the north row), 0.25 m between the platform and the pick face (south row: north face; north row: south face; the face is the one whose slots are `pickable`). `--along` gives the first design of step 6: beside the shelf, bow along the aisle, yaw 0 |
| `receiving_table`, `packing_table`, `bin_1`, `bin_2` | on the side of the bench that faces the storage zone, bow towards the bench, 0.25 m from the bench top: (-3.75, 0) and (3.75, 0). A bin uses the station of its bench |
| `P1`..`P4` | on the door line, yaw 0 |
| `receiving`, `storage`, `packing` | the point of the zone farthest from any obstacle at the LiDAR height: (-4.5, -2.5), (1.7, 2.8), (4.5, 2.5) |

The shelf stations stand beside the shelf, not in front of it, because a turn on the spot there is not possible with any margin. Smallest distance between the platform polygon and an obstacle, computed on the SDF:

| Station | At the pose | During a full turn on the spot |
|---|---|---|
| Shelf, bow along the aisle (`--along`, gap 0.25) | 0.250 m | 0.094 m |
| Shelf, bow towards the shelf (default, gap 0.25) | 0.250 m | 0.179 m |
| Shelf, bow towards the shelf (gap 0.30) | 0.300 m | 0.229 m |
| Bench | 0.300 m | 0.229 m |
| Zone points | 0.84 .. 1.03 m | 0.73 .. 0.96 m |

The results below, 36 goals, are for the first design (bow along the aisle, `--along`): neighbouring units need no turn, and to stop with yaw 0 coming from the other side the robot reverses. The reach analysis of step 7 then chose the bow towards the shelf, which is now the default (see step 7).

`nav_semantic.py --tour all --repeat 3` visits the seven shelf units, the two benches and the three zones, three times: 36 goals, all reached, 3 recoveries in total (S1 twice, `packing_table` once). Final pose against Gazebo's ground truth:

| Places | goals | position error [m] mean / max / std | yaw [deg] mean / max | Nav2's own estimate [m] mean / max | above 0.10 m |
|---|---|---|---|---|---|
| Shelf units | 21 | 0.079 / 0.163 / 0.038 | 3.2 / 6.3 | 0.062 / 0.123 | 7 of 21 |
| Benches | 6 | 0.063 / 0.094 / 0.029 | 2.6 / 8.4 | 0.030 / 0.075 | 0 of 6 |
| Zone points | 9 | 0.138 / 0.194 / 0.036 | 4.1 / 6.4 | 0.124 / 0.174 | 7 of 9 |
| All | 36 | 0.091 / 0.194 / 0.046 | 3.3 / 8.4 | 0.072 / 0.174 | 14 of 36 |

Moving between neighbouring shelf units takes 5-10 s (path about 1.2-1.5 m); the long runs are 11-13 m to the zone points (32-37 s). The error follows the quality of Nav2's own estimate: it is best at the benches (0.03 m) and worst at the zone points (0.12 m), which are by construction the points farthest from any obstacle. Path length alone does not explain it: `receiving` and `packing` come after 11-13 m, but `storage` after only 3 m and still has the largest errors (0.125, 0.155, 0.194 m). The common factor is probably the little geometry near those points; this was not tested.

### Known limits (step 6, so far)

- In the RViz view the robot ended the mapping run away from the start point, so loop closure was probably not exercised; the map check shows no error above 9.5 cm anyway.
- The inside of shelves, benches and columns is unknown in the map (the LiDAR sees only the faces). In the Nav2 costmap that interior looks free: the map is saved in trinary mode with free threshold 0.25, and Nav2 seems to load unknown pixels as free. It is harmless because those cells are enclosed by occupied ones.
- The travel pose leaves 19 mm at the rear: a joint error (stiction at rest up to about 3 mrad on elbow, wrist1 and wrist2, about 0.5-1 degree on wrist3) uses part of it.
- Nav2 does not yet refuse to start if the arm is not in the travel pose.
- Obstacles closer than 0.2 m to the LiDAR are ignored by SLAM, AMCL and the costmaps; the sensor sits 6 cm behind the front edge of the platform.
- The final pose is limited by the localization, not by the controller: 4 of the 18 goals ended more than 0.10 m from the goal in Gazebo's ground truth (the goal checker tolerance), because Nav2's own estimate was off by up to 0.14 m. The estimate is worst around the south-west of the receiving zone (0.10-0.14 m at the "receiving south" goal).
- Collisions are not measured automatically: no goal failed and there was one recovery in 18 goals, but that is not a collision test.
- Behind the robot the LiDAR cannot see in a cone covered by the arm base (except a window of about 9 degrees on each side of the axis), so reversing relies on the map and on what the costmap already holds. The world is static.
- Nav2 and slam_toolbox must not run together.
- The pick stations at (+-3.8, 0) are too tight to turn in place (see above); open places will be needed for step 7.
- At the shelf stations 7 of 21 arrivals ended more than 0.10 m from the goal in the ground truth (mean 0.079 m). The test did not split the error along and across the aisle; across it means towards or away from the shelf, which matters for the 0.25 m gap. `nav_goto.py` now prints both.
- Crossing the aisle at constant heading (N1 to S1, 1.15 m) took 13.3 s and one recovery; it was done once.
- The stations of the benches are provisional; those of the shelves were chosen in step 7 from the reach of the arm.

## Step 7: MoveIt 2 (in progress)

Done: trajectory interface of the arm controller (7.1), MoveIt configuration (7.2), planning scene of the warehouse (7.3), reach analysis on the shelf slots (7.4). Still to do: the gripper in the sequence approach, grasp, retreat.

### Trajectory interface (7.1)

MoveIt executes through a `FollowJointTrajectory` action; the arm controller only had its point-to-point `~/goal`. Now:

- **`ArmModelController`** takes a `JointTrajectory` on `~/trajectory`. Between two consecutive points it uses the quintic through their position, velocity and acceleration (MoveIt's time parameterization gives all three), and feeds them to the computed torque as the reference. A trajectory needs at least 2 points, all three quantities on every point, increasing times and speeds below the limit of the controller (+10 %); it starts only when the arm is idle and its first point is within 0.05 rad of the reference (the goal of the last move) or of the measured pose of the arm. The second condition matters: MoveIt plans from the measured state, so an arm held away from its last goal (blocked, or stopped short by friction) used to get every later trajectory refused, for ever, because it could not reach a goal it was pushing against. `~/goal` works as before.
- **`follow_joint_trajectory_server.py`** (started with the simulation) forwards the trajectory, waits for its end, checks the final error (0.03 rad if the goal gives no tolerance) and answers. A cancel is rejected: the controller cannot stop a trajectory half way.

Same move (travel to home90, 6 s) with `arm_goto.py` and through the action, largest tracking error [rad]:

| Joint | `arm_goto` | action |
|---|---|---|
| pan / lift / elbow | 0.0012 / 0.0004 / 0.0012 | 0.0013 / 0.0004 / 0.0011 |
| wrist1 / wrist2 / wrist3 | 0.0012 / 0.0031 / 0.0059 | 0.0012 / 0.0031 / 0.0059 |

Peak torques are the same (about 27 N m on the lift, 25 N m on the elbow). A trajectory of only 3 points, with a non-zero speed in the middle, ends with `error_code 0` and the same errors.

### MoveIt configuration (7.2)

`amr_manipulator_moveit_config`: SRDF (group `arm`, the chain from `ur_base_link` to `gripper_tcp`; group `gripper`; named poses `travel`, `home90`, `open`, `closed`; planar virtual joint `map` to `base_footprint`, whose pose comes from TF), numerical KDL kinematics (IKFast in MoveIt covers the UR5, not the UR5e), OMPL, and the controller interface of 7.1.

- **Collision matrix generated, not written:** `collisions_updater` of MoveIt on 10000 random configurations: 54 pairs disabled (31 adjacent, 23 never colliding). Forearm, wrists and upper arm against the chassis stay checked.
- **Limits:** 1.5 rad/s and 2.0 rad/s^2 on every joint. The speed stays below what the controller accepts; the acceleration is a first value, to be checked against the torques. `allowed_start_tolerance` is 0.03 rad because the wrists rest 3-9 mrad off the reference (stiction).

| Check | Result |
|---|---|
| Forward then inverse kinematics, 50 random configurations, seed up to 0.3 rad away | 50 of 50, pose error 0.010 mm and 0.0004 deg, 1 ms each |
| Joint goal to `home90`, planned and executed | 3.3 s, largest joint error 0.0059 rad |
| TCP down by 10 cm, IK seeded with the current joints and a joint goal | largest joint change 0.26 rad, TCP error 2.6 mm, 0.37 deg |
| TCP up by 10 cm on a straight line | 100 % of the line, 12 points, TCP error 0.6 mm, 0.09 deg |
| TCP by (+10, +10, 0) cm on a straight line | 100 %, 14 points, largest step 0.056 rad, TCP error 0.3 mm, 0.46 deg |

**A pose goal in MoveIt's own way picks a far IK branch.** MoveIt samples the solution of a pose goal from random states, not from the current one. On 24 goals within 15 cm of `home90`, planned only: 21 were planned, and **19 of them changed a joint by more than 1 rad** (median 5.86 rad, largest 6.64); in the first run on the robot a move of 10 cm ended with wrist1 at -5.4 rad, and the way back was refused (`INVALID_MOTION_PLAN`: the time-parameterized path went through states in collision). So the arm is moved in two ways: an IK solution seeded with the current joints followed by a joint goal (far branches are refused, `--max-jump`), and `compute_cartesian_path` with a jump threshold for straight approach and retreat. The joint limits stay at the +-2 pi of the URDF: restricting them would remove the multiples of 2 pi but not the eight branches of a UR5e.

### Planning scene (7.3)

`scene_from_sdf.py` turns the SDF into collision objects in the frame `map`: one object per model (walls, shelves, benches, bins, columns), a box for every box collision, at every height: 14 objects, 75 boxes. It is applied once `move_group` is up (launch argument `scene`). The articles are not in the scene yet.

Checked with `check_state_validity`, where the pose of the base is given in the request (no simulation needed):

| Base pose, arm | Result |
|---|---|
| (0, 0), `travel`, whole robot | valid |
| (0.6, -0.335) = station of S3, `home90`, whole robot | valid |
| (0.6, -0.8), `travel`, whole robot | in collision: chassis, right wheel, upper arm and wrists against `shelf_S3` |
| (0.6, -0.335), arm horizontal towards S3 with the elbow lowered, arm only | in collision: forearm, wrists, gripper and camera against `shelf_S3` |

The same check on the running simulation (robot at the origin, whole robot at base pose (0.6, -0.8)) gives the same contacts. That the planners respect the scene was shown with a test obstacle, a 6 cm wall 12 cm ahead of the TCP across a straight line of 20 cm from `home90`: the straight-line path is complete without the wall, 25 % long with the wall and the collision check on, and 100 % with the check off.

A straight line sideways from `travel` stops at 42 % with the check on or off, with the scene or without it: that is the reach of the arm, not a collision. A test of "stops before the shelf" from the origin therefore proves nothing: the shelves are 0.9 m from the robot and the arm cannot get there from the middle of the aisle.

### Reach analysis (7.4)

`scripts/reach_check.py` asks MoveIt, for every pickable slot and every station design, whether the arm can **approach and reach** the article: a collision-free IK solution of the pre-grasp pose (10 cm before the grasp, against the approach direction, with the planning scene of 7.3 and the base at the station pose), and from there a straight line to the grasp pose that goes 100 % of the way, free of collisions. The tool z axis is the approach direction and the fingers close along its y axis. Two kinds of grasp:

- **top**: from above, fingers closing along the aisle (along the depth for an article too wide along the aisle, as the first parcel was);
- **front**: horizontal, into the shelf, fingers closing along the aisle, tool x up (the camera on top).

Station designs, as in step 6: robot beside the shelf with the bow along the aisle (`along`), or with the bow towards the shelf (`facing`); the gap is the free distance between the platform and the shelf face.

**The tool is checked in free space first.** From the middle of the aisle, with the tool down, the arm reaches z 0.2 to 0.8 m at 0.4-0.7 m from its axis, and at z = 1.0 m only 0.3 m from it; with the tool horizontal it reaches z 0.4 to 1.2 m at 0.4-0.6 m. So the results below are limits of the arm and of the shelves, not of the tool.

**The grasp from above is not possible on the shelves: 0 of 28 real articles, at every gap and orientation tried.** At levels 1 and 2 the board above leaves 0.27 m, not enough for the gripper; at level 3 the pre-grasp pose is at z of about 1.09 m, out of reach of an arm whose shoulder is at 0.41 m.

**The front grasp works.** Real stock (28 articles on the pickable faces, 4 of them parcels; the whole warehouse has 15 parcels in 69 articles), front grasp:

| Design | Gap [m] | Articles reached |
|---|---|---|
| facing | 0.05 / 0.10 / 0.15 / 0.25 / 0.35 | 24 / 24 / 24 / 24 / 23 of 28 |
| along | 0.05 / 0.10 / 0.15 / 0.25 / 0.35 | 24 / 24 / 23 / 23 / 22 of 28 |

The ones that were never reached are the **parcels**: 4 of 4 at every gap (and, at some gaps, a jam jar). That parcel was 0.10 x 0.07 x 0.04 m: 0.04 m high, so its centre was 0.02 m above the board while the body of the gripper is 0.06 m thick (the gripper would have touched the board); and turned by the 15 degrees of jitter it measured 0.115 m along the aisle, against an opening of 0.085 m (the first check of the sizes looked only at the smaller side). The parcel was redefined as 0.06 x 0.08 x 0.09 m (0.079 m along the aisle at 15 degrees, 4.5 cm of centre height); the check of the sizes was corrected, and the analysis repeated with the new one:

| Design | Gap [m] | Articles reached (28, parcels included) |
|---|---|---|
| facing | 0.05 / 0.15 / 0.25 / 0.35 | 28 / 28 / **28** / 27 |
| along | 0.05 / 0.15 / 0.25 / 0.35 | 28 / 27 / 27 / 25 |

What is still missed is a jam jar (0.075 m high, just above the minimum of 0.07 m) at some gaps and, at gap 0.35 beside the shelf, a juice carton and a parcel.

Envelope of all 105 pickable slots with a generic 0.17 m tall box, front grasp, robot centred on the unit:

| Design | Gap [m] | Slots reached of 105 |
|---|---|---|
| along | 0.05 / 0.10 / 0.15-0.30 / 0.35 / 0.45 | 105 / 104 / 98 / 85 / 56 |
| facing | 0.05 / 0.10 / 0.15 | 98 / 102 / 103 |
| facing | **0.20 / 0.25 / 0.30 / 0.35** | **105 / 105 / 105 / 105** |
| facing | 0.45 | 90 |

With the bow towards the shelf every slot is reachable over a band of gaps (0.20-0.35 m). With the bow along the aisle the first slots of level 3 are lost at every gap above 0.10 m.

**Tolerance to the arrival error** (the error of step 6 was 0.08 m on average and 0.19 m at most): facing stations with gap 0.25 and 0.30, 15 errors each: 0.10 and 0.15 m along the aisle, 0.05 and 0.10 m towards and away from the shelf, 5 degrees, and combinations.

- **Levels 1 and 2: 35 of 35 in every one of the 30 cases.**
- Level 3 is the sensitive one. Gap 0.25: from 28 to 35 of 35 (worst: an error of 0.15 m along the aisle, or 0.10 m along the aisle together with 5 degrees). Gap 0.30: from 21 to 35 of 35 (worst: 0.10 m farther from the shelf). The slots that fail are those at the end of the unit, on the side the robot has drifted from.
- The inverse kinematics restarts from random states: the same case gave 104 and 105 of 105 on two runs, so the level 3 numbers carry a variation of one or two slots.

**Chosen station:** bow towards the shelf, centred on the unit, gap 0.25 m (the middle of the band where all slots are reachable, and the one with the smaller loss under arrival errors). It is the default of `semantic_goals.py` and `nav_semantic.py` (`--along` for the other design).

Station design, navigation margin (smallest platform to obstacle distance in a full turn on the spot, SDF geometry):

| Station | At the pose | In a full turn |
|---|---|---|
| along, gap 0.25 | 0.250 m | 0.094 m |
| facing, gap 0.25 | 0.250 m | 0.179 m |
| facing, gap 0.30 | 0.300 m | 0.229 m |

**Navigation test of the chosen station** (`nav_semantic.py --tour shelves --repeat 3`, Nav2 with the saved map): the seven shelf units three times, 21 goals, all reached, 5 recoveries (S4 once, N2 once, N1 three times), 9.6 to 18.4 s per goal on average against 5 to 10 s with the bow along the aisle (two turns per move).

| Final pose against Gazebo's ground truth | mean / max / std |
|---|---|
| Position [m] | 0.078 / 0.122 / 0.025 |
| Towards or away from the shelf [m] | +0.000 / 0.088 / 0.042 |
| Along the aisle [m] | -0.048 (signed, in the frame of the goal) / 0.121 / 0.051 |
| Yaw [deg] | 2.5 / 8.3 / 2.0 |
| Nav2's own estimate against the truth [m] | 0.039 / 0.094 / 0.021 |

The error towards the shelf stays within 0.09 m (all inside the 0.10 m of the tolerance sweep); along the aisle it reaches 0.12 m (5 goals of 21 above 0.10 m, none above 0.15 m), where the sweep keeps levels 1 and 2 complete and loses some end slots of level 3. In both rows the robot ends to the right of its heading (-5.6 cm on the south row, -3.7 cm on the north row, in the frame of the goal), so the offset is systematic rather than random. The mean position error (0.078 m) is the same as with the bow along the aisle (0.079 m); the largest is lower (0.122 against 0.163 m).

### Known limits (step 7, so far)

- **The arm stopped following trajectories twice** while `make_dataset.py` photographed the shelves with the robot (21 and 36 images into two runs): the first error was `ur_wrist_3_joint ended 0.7893 rad from the goal` in one run and `ur_wrist_1_joint ended 0.0346 rad` in the other, then every trajectory was refused. The refusal is explained and fixed (the controller compared the first point with the last goal, not with the arm; `scripts/traj_recovery_test.py` fails with the old condition, the arm 0.057 rad from its goal, and passes with the new one; the tracking test is unchanged, 0.006 rad at most). **What caused the first error is not known**: three later runs (about 25 look poses with and without fillings, and a full run of 64 images, logged at 50 Hz) did not reproduce it; the error at rest was at most 0.020 rad (wrist 3, the weakest joint) and in motion 0.027. A contact that MoveIt does not see is a guess, not a result. The real time factor drops to 0.04-0.2 for a few seconds around the fillings (median 0.78), and the action server waits in wall-clock time.
- The articles are not in the planning scene: neighbours of the article to grasp, and the article itself, are not checked as obstacles. The fingers open to 0.085 m between articles 0.20 m apart.
- The minimum height of an article (0.07 m) is a little optimistic: a jam jar of 0.075 m is missed at some gaps.
- The reach analysis uses the pose of every slot as the stock was generated (articles within 1 cm and 15 degrees of the nominal pose); the real position will come from the perception of step 8.

- The orientation of the TCP ends 0.1-0.7 degrees off after a move: it is the stiction offset of the wrists.
- The acceleration limit of 2 rad/s^2 is a first value, not checked against the measured torques.
- The controller cannot stop a trajectory half way, so the action cannot be cancelled.
- The scene is static and has no articles. The planners were not yet tried with the robot standing at a shelf station (that needs Nav2 running together with MoveIt); that scene objects are respected was shown with a test obstacle (see above).
- MoveIt's native pose goals are not used (see above).

## Step 8: perception (in progress)

Done: stereo pair, stereo depth, observation poses (8.1), labels from the simulator, the synthetic dataset of 3990 images and the recognition network for the six articles (8.2). Still to do: the pose of the target from the mask and the stereo depth, then the grasp sequence (8.3).

### Stereo pair and observation poses (8.1)

**Cameras.** With the simulation argument `stereo:=true` two cameras are added in the housing of the wrist camera, 25 mm either side of its centre (baseline 50 mm), with the intrinsics of the RGB-D camera (320 x 240, field of view 1.2 rad, fx = 233.9 px) and a little image noise (standard deviation 0.01, assumed, not calibrated). The left one is an `rgbd_camera`, so it also gives the **true depth in its own frame**, the reference to measure the stereo depth against; the right one is a plain camera. The images are rectified by construction (pinhole, no distortion). Topics: `/stereo/left/{image,depth_image,camera_info}`, `/stereo/right/{image,camera_info}`; frames `stereo_left_optical_frame` and `stereo_right_optical_frame`. The RGB-D camera of step 4 is unchanged, apart from its mounting: it is rolled by 180 degrees about its optical axis (`rpy` of `camera_joint`), because its up axis is the x axis of the tool with a sign that left the images upside down when the tool is horizontal with its x axis up, the pose used to look at a shelf (the camera sits on top of the gripper). Checked by computing the image axes in that pose: before the roll the image down axis pointed up in the world, now it points down; and on the images of the simulation.

**Depth.** `amr_manipulator_perception/stereo_depth_node.py` matches the two images with OpenCV's semi-global block matching (128 disparities, so the nearest depth is 0.09 m, below the 0.1 m near clip; blocks of 5 pixels; a border on the left of both images, otherwise the first 128 columns have no depth) and publishes `/stereo/depth` (32FC1, NaN where there is no match) in the left frame. **It works on demand**: it keeps the last pair and computes when `/stereo_depth/compute` (a `Trigger` service) is called, or at a low rate with the parameter `rate`.

Real time factor, simulation with the warehouse and no GUI (four readings each, 4 s apart):

| Configuration | Real time factor |
|---|---|
| No stereo | 0.98 - 1.01 |
| Two more cameras | 0.86 - 1.03 |
| Cameras and node on demand (idle) | 0.93 - 1.00 (0.75 in the first reading, right after the start) |
| Cameras and node computing at the rate of the cameras (10 Hz) | **0.29 - 0.35** |

The matching at 10 Hz takes so much CPU that the simulation falls to a third of real time: hence the on-demand node.

**Accuracy**, measured by `scripts/stereo_check.py` against the true depth of the left camera, pixel by pixel, 5 frames. Robot at the station of shelf S3 (driven there, 0.25 m from the aisle), camera 0.38 m from the soup can of slot `S3.L2.n4`. The pixels are split by texture (local deviation of the grey image above 6 grey levels, the noise being about 2.5):

| True depth, pixels with texture | Pixels with a stereo depth | Bias [mm] | Median / 95th percentile of the absolute error [mm] |
|---|---|---|---|
| 0.30 - 0.50 m | 98 % | +0.1 | 8.3 / 31.6 |
| 0.50 - 1.00 m | 88 % | -66.2 | 21.2 / 557 |

Below 1 m, stereo gives a depth on 86 % of the textured pixels and on 46 % of the others, where it is wrong by tens of millimetres (median 34 mm between 0.3 and 0.5 m). In the working range the median error, 8.3 mm, matches what the geometry gives: at 0.4 m a disparity error of 0.5 pixel is 6.8 mm (depth error = depth^2 x disparity error / (fx x baseline)). The bias changed from +0.1 to +8 mm between runs and is not calibrated.

**Observation poses.** `moveit_check.py look --slot S3.L2.n4 --dist 0.30` puts the TCP 0.30 m in front of the centre of the article, tool horizontal into the shelf: IK seeded near that posture (a few tries each; the solver restarts from random states, so a solution can come with a joint turned by a multiple of 2 pi, and every solution is brought to the equivalent angles closest to 0 that the joint limits allow; the one with the smallest largest joint is taken) and a MoveIt joint goal. A first version rejected every joint beyond +-3.3 rad instead of normalizing it, and failed from the station with the wrist 2 at -4.2 rad, the same posture as +2.1. The camera is 0.08 m behind the TCP and looks along the tool z axis: with `--dist 0.30` it is 0.38 m from the centre of the article. From the station of S3 the arm went from the travel pose to the look pose of slot `S3.L2.n4` in 7.9 s (10 collision-free solutions found, the chosen one with a largest joint of 2.14 rad). Reach of the observation pose, bow towards the shelf, gap 0.25 m, all 105 pickable slots, with `reach_check.py --grasps look`:

| TCP distance to the article centre | Camera distance | Slots of 105 |
|---|---|---|
| 0.15 m | 0.23 m | 105 |
| 0.30 m | 0.38 m | 97 (the centre slots of level 1 are lost) |
| 0.45 m | 0.53 m | 84 |

### Dataset (8.2)

The network is built from scratch, as a learning exercise (no pretrained models); its pose output is not learned: the pose of an article comes from the mask, the stereo depth and the known shape of the article.

**Labels.** Gazebo's segmentation camera gives the class of every pixel. Every article model carries a `Label` plugin with the class id (`label` in `articles.yaml`: cereal_box 1, juice_carton 2, soup_can 3, water_bottle 4, parcel 5, jam_jar 6; 0 is the background), written by `make_articles.py`. With the simulation argument `dataset:=true` a segmentation camera is added at the pose of the left camera of the stereo pair, with the same intrinsics, so the labels are aligned with the left image. Topics `/segmentation/labels_map` and `/segmentation/colored_map` (the label is in the first channel of an `rgb8` image). Checked with `scripts/label_probe.py` from the station of S3, looking at `S3.L2.n4`: three classes found in the view (cereal_box, soup_can, jam_jar), the masks follow the outline of the articles, even on the white body of the soup can that has little contrast against the board. The real time factor with the segmentation camera and the stereo pair was 0.96.

**Filling the shelves at random.** One stock of 69 articles would be learned by heart, so the articles are created and removed in the running simulation (`scripts/stocker.py`, Gazebo's `create` and `remove` services, static articles). A call takes 0.34-0.37 s; 20 calls in parallel take about 2.4 s whether 4, 8 or 16 go at the same time (the queue is in the server), and the 20 articles were all really in the world. On the images: a parcel created on an empty slot appears with class 5 (about 3100 pixels), one created outside the field of view does not appear, and after the removal the labels are the same as before.

**Generator.** `scripts/make_dataset.py` works in the running simulation (`stereo:=true dataset:=true`, Nav2, `move_group`). It removes the 69 articles of the warehouse (restart the simulation to get them back), then, for every shelf unit: Nav2 takes the robot to its station (the arrival error of 5-12 cm is a variation the dataset should have), the arm goes to a look pose (a random slot of the pick face, a TCP distance of 0.15-0.40 m, a random offset of up to 3 cm; a target out of reach is replaced by another, up to 4 times), and for every look pose several random fillings are photographed: each slot of the row within 0.4 m of the camera, on both faces and all levels, is filled with a probability drawn between 0.2 and 0.9, with a random class (uniform over the six), a yaw of up to 15 degrees and a shift of up to 1 cm. Per sample it saves `images/NNNNNN.png` (left image), `labels/NNNNNN.png` (class id per pixel) and `meta/NNNNNN.json` (look pose, fill, camera pose in the map at the time of the frame, every article created with its class, slot and pose in the map, pixels per class). **The split is by look pose, not by image** (70, 15 and 15 percent, decided by a hash of the pose id): all the images of a pose go to the same split, so validation and test never contain a camera pose seen in training. `scripts/dataset_sheet.py` makes a contact sheet of a split, with the labels painted over the images. The dataset is written in `dataset/`, which is ignored by git.

Two defects of the first version were found by looking at the images and the metadata, not by the code: 4 images of 6 had no articles although 12 to 22 had been created, and the pose of the camera of one look pose changed from 0.712 to 0.424 m between images. The node that grabs the frames uses the simulation time, which moves only while the node spins, so the frames queued before the filling were accepted as new; and the pose of the camera was read as the latest transform of a buffer that was behind. Now the node spins for a second, takes the newest stamp as reference and accepts only a later frame, and the pose is read at the time of the frame. The same short test (two shelf units, 2 look poses and 3 fillings each) then gave 12 images, no failure, and 11 of them with articles in view.

Time: about 10 s per image in that short test, which includes the removal of the original articles, the navigation and the look poses (a filling costs about 5 s: 20 articles are created in about 2.4 s, the same to remove them). For the first target of 3000 images for training and 500 each for validation and test (4000 in all) this is of the order of 8 hours; it is an estimate, to be checked on a first batch.

**A camera without the robot.** The robot is what makes the generator slow: the navigation, the planning and 8 s to move the arm for every look pose. The images do not need the robot, only a camera with the same sensors: `models/dataset_camera` is a free camera (a kinematic model, no gravity) with the left camera and its true depth, the right camera and the segmentation camera, with the same topics and the same intrinsics as the robot; `launch/dataset.launch.py` starts the warehouse, that camera and the bridge, nothing else (no robot, controllers, Nav2 or MoveIt), and `stocker.set_pose` moves the camera with a service of the world. The gripper is not modelled on the camera. That was a mistake: the camera is 0.08 m behind the tool point and the fingers end 0.09 m in front of it, that is 0.17 m from the camera, beyond the near clip of 0.1 m, and every image of the robot has a fixed light shape in the bottom-left corner, most likely a part of the gripper (see the test with the robot below). The images of the free camera do not have it.

**Generator without the robot.** `scripts/make_dataset_free.py` (the code shared with `make_dataset.py` is in `scripts/dataset_common.py`). Start `ros2 launch amr_manipulator_gazebo dataset.launch.py`, then run the script with `--out dataset`. An *arrangement* is a random filling of one row (S or N) around a random centre, with the rules above (half width 0.7 m, fill drawn between 0.2 and 0.9, uniform classes, yaw up to 15 degrees, shift up to 1 cm); each arrangement is photographed 20 times (`--shots`). The camera looks at the picking face of the row, perpendicular to it, aiming at a random slot near the centre of the arrangement: 0.23-0.53 m from the slot (the TCP distance of the arm, 0.15-0.40 m, plus the 0.08 m of the camera), at the height of a shelf level (0.3, 0.6 or 0.9 m plus 0.085 m, to the centre of an article, plus or minus 5 cm), with a random error of up to 1 degree of roll, 2 of pitch and 6 of yaw, which is about what the arm does. **The split is by arrangement** (70, 15 and 15 percent, decided by a hash of the id): the validation and test images show fillings that the network has never seen, from any pose. The metadata give the true pose of the camera (the commanded one, `camera_in_map`), the articles created with class, slot and pose, and the pixels per class.

Measured on a first small batch (12 arrangements, 5 shots each): 60 images in 121 s, **2.0 s per image** (this includes the removal of the original articles and a filling every 5 images), no failure, 8, 3 and 1 arrangements in train, validation and test (40, 15 and 5 images), every class present in the training and validation images (the test split, one arrangement of 5 images, has no parcel: with so few arrangements it can happen, and it is to be checked on the statistics of the full dataset), the background 90 percent of the pixels in the training images. The first image after each filling, which is the one that can be taken too early, had its articles in all 12 cases, and the painted labels coincide with the articles in the images. **A first full run that was wrong, and what it taught.** The first run of 200 arrangements of 20 shots was stopped after 5979 s: the real time factor fell from 1.0 to 0.14, the time per image grew from 0.9 s to 3.4 s and the `set_pose` calls timed out (948 shots lost). Of its 1711 images, 765 had labels of classes that their arrangement had not created (from image 289 on, 174 of 200 around image 1300), and an arrangement that should have created about 30 articles created 7: articles of earlier arrangements were left in the world, and a validation image could show articles of a training arrangement. That dataset was deleted. A test of 15 cycles of 30 creations and 30 removals then showed that **the answers of the `create` and `remove` services cannot be trusted** (from cycle 9, up to 5 of 30 removals were answered as failed) **while the scene of the world is always right** (the 30 articles there, then none). So the generator now reads the scene (`/world/amr_world/scene/info`, 0.33 s) after every removal and filling, repeats what is missing, and drops the arrangement with an error if the world cannot be put right; the images of an arrangement are kept in memory and saved only if the scene at the end still holds exactly the articles of the metadata, and a label of a class that was not created is an error. The generator leaves with code 75 when the world is too slow (real time factor below 0.6) or cannot be put right, and `scripts/generate_dataset.sh` then restarts the world and goes on, in batches, until the number of arrangements is reached.

**The dataset (generated).** 200 arrangements, 7 batches of the script, 3990 images of 320 x 240 (4000 minus 10 shots lost because the camera could not be moved), 253 MB, 7927 s of generation (2.0 s per image, the restarts of the world not included). Six batches were ended by the generator itself, between the 22nd and the 35th arrangement, because the scene of the world could no longer be read; no arrangement was saved in a bad world. The time per image still grows inside a batch (from about 1.1 s to 1.9 s) although the scene is right, so the leftover articles were not the only cause of the slowdown; what grows is not known (it may be the cost of creating and removing entities in Gazebo) and the restart is a workaround, not a cure.

| split | arrangements | images | images without articles |
|---|---|---|---|
| train | 134 | 2675 | 80 |
| validation | 33 | 657 | 21 |
| test | 33 | 658 | 16 |

Checks on the files (read-only scripts, not in the repository): every image has its label and metadata, no orphan or corrupt file, no duplicate image, no label of a class that the arrangement did not create. Every class is in every split (train: cereal_box in 1464 images, juice_carton 1529, soup_can 1423, water_bottle 1289, parcel 1483, jam_jar 1270). The background is 91 percent of the pixels in train and validation and 90 percent in test; the rarest class in train is the jam jar (0.81 percent of the pixels), the others are between 1.16 and 2.2 percent. The split by images is 67, 16 and 16 percent, because it is by arrangement. `make_dataset.py` was used to take a few images with the real camera of the robot, to test the network on a view it has not been trained on (below).


### Recognition network (8.2)

`amr_manipulator_perception/segmentation/`: a U-Net written from scratch in PyTorch (`model.py`), without pretrained weights. A block (`ConvBlock`) is two 3x3 convolutions, each followed by BatchNorm and ReLU; the network has four downsamplings (max-pooling and a block) and four upsamplings (transposed convolution, concatenation with the skip connection of the same size, a block), and a 1x1 convolution that gives the scores of the 7 classes (background and the six articles) for every pixel. Widths 16, 32, 64, 128 and 256, 1.94 million parameters, input 240 x 320. `check_model.py` checks the shapes of every block, `sanity.py` checks the whole chain on 8 fixed images (loss from 2.22 to 0.10 in 300 steps, mean IoU 0.990 on them).

`train.py`: cross entropy without class weights, Adam with a learning rate of 1e-3 and a cosine decay, batches of 8, 30 epochs of 39 s each (1168 s) on the 4 GB GPU of the laptop. The augmentation is applied to the training images only: a shift of up to 16 x 12 px (the same for image and labels, nearest neighbour for the labels), contrast, brightness, a little blur and noise. The network kept is the one with the highest mean IoU on the validation split (epoch 29). The measure is the IoU of every class (`metrics.py`), not the accuracy of the pixels: answering "background" everywhere is right on 91 percent of them.

Test split (658 images, never used before the end):

| class | IoU |
|---|---|
| background | 1.000 |
| cereal_box | 0.997 |
| juice_carton | 0.992 |
| soup_can | 0.986 |
| water_bottle | 0.996 |
| parcel | 0.995 |
| jam_jar | 0.992 |
| **mean** | **0.994** |

The validation mean is 0.995. In thirds of the camera distance the mean IoU is 0.992, 0.996 and 0.994: the far shots, where the articles are a few pixels wide, are not worse.

**Test with the robot.** The same network on 36 images taken with the real camera of the robot (`scripts/robot_testset.sh`: Nav2 to the station, the arm to a look pose, random fillings, as `make_dataset.py` does): mean IoU **0.960**.

| class | IoU | IoU without the pixels under the shape |
|---|---|---|
| background | 0.996 | 0.998 |
| cereal_box | 0.994 | 0.994 |
| juice_carton | 0.971 | 0.997 |
| soup_can | 0.841 | 0.897 |
| water_bottle | 0.996 | 0.998 |
| parcel | 0.955 | 0.992 |
| jam_jar | 0.969 | 0.969 |
| **mean** | **0.960** | **0.978** |

The shape is the one of the gripper in the corner: rows 204 to 239 and columns 52 to 114 of the image (2.2 percent of it), the same in every image. The segmentation camera does not see it, so the labels mark the articles behind it (2.4 percent of the article pixels, 12 of the 36 images) while the image shows the shape: the network answers background there, which is right for the image, and the labels count it as an error. The brightness of the images is the same as in the dataset (mean 97-99 against 92). The weakest class is the soup can, whose body is white or grey against a grey background and whose mask is often partial. The network was not trained again for the shape. A first attempt at this test was made by mistake in the empty world (the default of `sim.launch.py`) and was discarded; the script now passes `world:=warehouse.sdf`.

### Known limits (step 8, so far)

- Walls, boards and floor are plain colours, so stereo has no depth there (a real D435 has an infrared projector for this reason): the depth is to be used inside the mask of the article, where there are labels and stripes.
- Outside the textured regions the matches are wrong: the depth must be filtered with the mask of the recognition, not used as it comes.
- The noise of the images and the bias of the stereo depth are assumptions, not calibrated.
- The labels mark some articles that the image does not show, hidden by a light surface: 0.15 percent of the article pixels in the test split (a count of the pixels on near-white surfaces, so a lower bound) and 2.4 percent in the robot images, behind the shape. The cause was not investigated.
- The test with the robot has 36 images, a small sample; the figure is an indication, not a guarantee.
- Training, validation and test come from the same warehouse: they differ by arrangement and pose, not by shelves, walls or lighting. The test measures the recognition of unseen fillings, not of another environment.
- The stereo node is on demand: it gives one depth image per request, not a stream.
- The first reading of the real time factor after the start is lower; the figures above are from runs of a few minutes, not averages over a long time.

## Roadmap

- **Step 7 (rest):** the gripper in the sequence approach, grasp, retreat.
- **Step 8 (rest):** pose of the target from the mask of the network and the stereo depth, then the grasp sequence (first with the true pose, then with the estimated one).
- **Step 9:** task logic with BehaviorTree.CPP.
