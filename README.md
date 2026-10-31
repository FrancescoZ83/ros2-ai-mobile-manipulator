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
| 6 | slam_toolbox + Nav2 | in progress: travel pose, mapping, map check and navigation through the doors done; goals from the semantic map next |
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
amr_manipulator_gazebo/        launch files (simulation, SLAM), bridge.yaml, worlds, models, saved maps
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
- The LiDAR sees the UR5e base (a short sector behind the sensor); measured and filtered by range in step 6.
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
- The arm starts at the same pose as `home90`: the gripper is 0.64 m in front of `base_footprint` at 0.63 m, i.e. beyond the base. Driving into a bench at 0.3 m/s ended with the gripper hitting the bench top. The Nav2 footprint and a travel pose are needed before navigation (travel pose: see step 6).

## Step 6: SLAM and navigation (in progress)

Done: footprint and travel pose, LiDAR self-echo, mapping with slam_toolbox, numerical map check, localization (AMCL) and navigation (Nav2) through the four doors. Still to do: goals taken from the semantic map (6.6).

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

## Roadmap

- **Step 6:** goals from the semantic map (6.6).
- **Step 7:** MoveIt 2 with a `FollowJointTrajectory` interface on top of the arm controller.
- **Step 8:** object perception (PyTorch / ONNX).
- **Step 9:** task logic with BehaviorTree.CPP.
