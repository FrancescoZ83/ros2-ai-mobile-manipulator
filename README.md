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
| 8 | Perception (PyTorch / ONNX) | in progress: stereo pair and depth, observation poses, dataset and recognition network done (mean IoU 0.994 on the test split, 0.960 on 36 images of the robot's camera), pose of the articles measured in the simulation and on the robot's camera (2.0 mm median along the row on the robot), grasp with the true pose (29 of 30 held at one unit) and with the pose from the images (30 of 30), the choice of the branch of the pre-grasp solved level 3; the arrival of the robot with Nav2 measured (20 of 30 held, the localization along the shelf was the limit; 29 of 30 and 27 of 30 on new draws with the shift of the believed map found from the images; with the travel pose forward, the station at 0.30 m and the branch-first look: 29 of 30 and 27 of 30 again on two more sets of draws, and 28 of 30 and 30 of 30 with the station in front of the slot); the tilt of the cereal boxes next |
| 9 | Task logic with BehaviorTree.CPP | in progress: a mission (take an article from a shelf slot, carry it to the packing table, drop it in a bin) run by a behavior tree on skills of a Python server; 10 of 10 missions on one draw, the retries and the other bin tried with injected failures; one article lost in the drive seen; missions with two articles (two bins): 5 of 6 |

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
amr_manipulator_task/          task logic: the actions of the skills, the behavior tree (BehaviorTree.CPP 3) and its executor
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
# Compare the saved map with the boxes of the SDF world (no ROS needed)
python3 scripts/map_check.py [--tol-m 0.10]
# Pose and clearance of every place of the semantic map (no ROS needed)
python3 scripts/semantic_goals.py [S3 packing_table S1.L3.n5] [--gap 0.30] [--along]
# MoveIt (move_group running): kinematics, joint and pose goals, straight lines, collisions at any base pose
python3 scripts/moveit_check.py ik | joint --named home90 | pose --dz -0.10 | cartesian --dz 0.10
python3 scripts/moveit_check.py valid --whole --base 0.6 -0.335 0 --named home90
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
- **Travel pose** `[0.74, -1.72, -0.69, -0.54, -2.36, 1.66]`: the arm folded upright over the chassis, the tool horizontal and the camera looking forward: the tool z axis is (0.991, -0.046, -0.128), forward and 7 degrees down, and the camera is at (-0.065, -0.139, 1.134) m, so the robot sees where it is going. The first travel pose was `home90` with the base joint turned by 180 degrees (the arm folded low over the chassis, the camera looking down and back); it was replaced for that reason. The hull of the whole robot is still **identical** to the platform hull. The arm and the tools span x -0.248 .. +0.370, y -0.295 .. +0.183 and reach z = 1.234 m; the smallest margins are 30 mm at the front and 20 mm at the side. Found by an optimisation on the forward kinematics of the URDF (the camera axis forward and level or slightly down, the camera above 0.45 m, every link inside the platform, nothing at the height of the LiDAR plane, 0.275 m, in front of the arm base, no link in the field of view of the camera) and then checked in the simulator. The centre of mass of the robot (77.6 kg, 50 of them the chassis) is at (0.012, -0.005, 0.293) m, well inside the support (wheels at y +-0.29, casters at x +-0.3); it was at (0.013, -0.019, 0.269) m with the first pose.
- **Stow at launch.** Once all controllers are active, `sim.launch.py` runs `scripts/arm_goto.py` towards the travel pose (argument `stow`, default `true`). The script is found relative to the launch file, so this needs `colcon build --symlink-install`. The arm first ramps to the start-up pose (`home90`) and then goes to the travel pose in 6 s, a straight line in joint space; start in free space, or use `stow:=false`. After the stow the joints are within 0.011 rad of the goal (wrist 3 +0.011, the others 0.005 or less).

| Check | Result |
|---|---|
| Tracking error, 12 s move to the travel pose (from the first one) | max 0.0431 rad (wrist 3, it stays 0.043 rad off: it only turns the camera about its axis), other joints <= 0.0093 rad |
| Peak torque in that move | 27.0 N m (lift), 26.7 N m (elbow) |
| Collisions of the whole robot, the pose and 8 configurations on the way from the first travel pose | all valid (MoveIt, `moveit_check.py valid --whole`) |
| LiDAR returns of the robot | the same 28 rays as before (the base of the arm), nothing new |
| Tour of the seven shelf units with Nav2, arm in the new pose / in the first one | 7 of 7 reached both times; final position error mean / max 0.071 / 0.115 m and 0.063 / 0.097 m, yaw 2.2 degrees (mean) both |
| Tracking error of the arm during that tour (largest, per joint) | 0.011 rad or less with the new pose; the first one 0.010 rad or less except wrist 3 at 0.063 rad |
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
| `S1`..`S4`, `N1`..`N3` | in the aisle in front of the pick face, bow towards the shelf (yaw -90 degrees on the south row, +90 on the north row), 0.30 m between the platform and the pick face (0.25 m in the first design, see 7.4) (south row: north face; north row: south face; the face is the one whose slots are `pickable`). `--along` gives the first design of step 6: beside the shelf, bow along the aisle, yaw 0 |
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
- The travel pose leaves 20 mm at the side and 30 mm at the front: a joint error (stiction at rest up to about 3 mrad on elbow, wrist1 and wrist2, about 0.5-1 degree on wrist3) uses part of it.
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

Same move (the first travel pose to home90, 6 s) with `arm_goto.py` and through the action, largest tracking error [rad]:

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

**Chosen station:** bow towards the shelf, centred on the unit, gap 0.25 m (the middle of the band where all slots are reachable, and the one with the smaller loss under arrival errors). It is the default of `semantic_goals.py` and `nav_semantic.py` (`--along` for the other design). **Changed to 0.30 m** after the measure of the posture below (`SHELF_GAP` in `semantic_goals.py`; the benches stay at 0.25 m, `TABLE_GAP`).

**Posture of the pre-grasp against the gap, with the second travel pose** (`scripts/gap_check.py`, move_group only, a generic 0.17 m tall box in the 105 pickable slots of the seven units, facing stations; all the collision-free IK solutions of the pre-grasp from a grid of 20 seeds, those whose straight line is complete, the choice of the picking code; "flipped" is the base joint beyond +-1.6 rad, the arm reaching over its back). Reached, of 105: gap 0.20 m **84** (the centre column, n3, is lost at every level), 0.25 m **98** (the centre slot of level 1 in every unit, 7 slots), 0.30 and 0.35 m **105**, 0.40 m **91** and 0.45 m **90** (the ends n1 and n5 of level 3). The centre slot at level 1 that is not reached at 0.25 m is the one where the jam jar had no collision-free look pose in the series with the true pose: it was the gap. The pre-grasp with the shortest line (`--best-pregrasp`) is flipped in 40 to 60 percent of the slots at every gap, with a swing from the look pose of 10 to 16 rad (median); the swing-aware choice (`--swing-aware`) brings the swing to 1.4-2.3 rad and the flipped ones, at 0.25 m, to 9 of 28, 6 of 35 and 18 of 35 at levels 1, 2 and 3. A look pose chosen with the base joint within +-1.6 rad (`look_solution` takes the one with the smallest largest joint, and that one is flipped in 4 to 18 of 35 slots) and the swing-aware pre-grasp: at 0.30 and 0.35 m no pre-grasp is flipped at levels 1 and 2 (0 of 35 at 0.30; 0 and 1 of 35 at 0.35), at 0.25 m 8 of 28 and 4 of 35; at level 3 it stays at 14 or 15 of 35 at every gap. At levels 1 and 2 the best line with the shoulder not turned over is as short as the best one (longer by at most 0.3 rad); at level 3 in 14 of 35 slots at 0.25 m and 8 of 35 at 0.30 and 0.35 m it is much longer (up to 24 rad), so there the flipped pre-grasp is the only good one: a limit of the arm at the height of the third level, not of the choice. The move from the travel pose to the look pose is 8 to 10 rad in the sum of the joint changes (median) at every gap, even with the nearest look pose (6.5 to 8 rad): it is the arm unfolding. Moving away from the shelf therefore removes the missing slots and the flipped postures at level 1, and costs the ends of level 3 above 0.35 m. The tolerance to the arrival error measured above (gap 0.30: from 21 to 35 of 35 at level 3, worst when the robot stops 0.10 m farther from the shelf) is the price. **Applied**: the gap of 0.30 m, the look pose within +-1.6 rad of the base joint (`look_solution`) and `--swing-aware` as the default of `pick_oracle.py` (`--no-swing-aware` for the ranking by the line only).

**The ends of level 3 and the arrival error along the aisle** (`scripts/line_blocked.py`, move_group and the static scene, the real classes and grasp_dz, the robot at the station of the unit moved along the aisle by the offsets). Two attempts of the Nav2 series (a water bottle at L3.n5, a cereal box at L3.n1) had no pre-grasp with a complete straight approach, with the shift found and the estimate right. The approach stops when the robot is off towards the outside of the unit, the slot being farther to the side: at L3.n5 with the water bottle (grasp_dz 0.04 m) from 40 mm (the line reaches 82, 55 and 36 percent at 40, 60 and 80 mm), with the same bottle at grasp_dz 0 and with the cereal box from 80 mm; at L3.n1 the mirror image. The slot n3 completes at every offset between -80 and +80 mm. Two causes: the forearm touches the shelf unit (`ur_forearm_link` against `shelf_S1`), or the arm cannot follow the line even without checking collisions (the limit of the reach, with the slot 0.4 m to the side of the base). The four IK solutions stop at the same fraction. With the station moved towards the slot by 0.12 to 0.32 m every line completes (n5 with the bottle and the cereal box, n1 with the bottle). `gap_check.py --align` (the station in front of each slot, units S1 and S3, 0.30 m) against the centre of the unit: at level 3 the pre-grasp with the shoulder turned over in 2 of 10 slots instead of 4 of 10, and the best line without it is never longer than the best one (up to 10.4 rad longer at the centre of the unit); at levels 1 and 2 none is turned over (level 2: 0 of 10 instead of 2 of 10). **Applied**: the station is in front of the slot to pick (`slot_station` in `semantic_goals.py`, `resolve` and `nav_semantic.py` accept `S1.L3.n5`; `pick_oracle.sh` with `SLOT=S1.L3.n5`, or `SLOT=draw` which draws the slot from the seed before the robot is driven there; `pick_oracle.py --slot`). The distance of the platform from the obstacles at the 105 stations is not less than at the stations of the units (0.30 m).

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

Done: stereo pair, stereo depth, observation poses (8.1), labels from the simulator, the synthetic dataset of 3990 images and the recognition network for the six articles (8.2). The pose of the articles from the mask and the stereo depth is measured in the simulation and on the robot's camera (8.3, first part). The grasp with the true pose of the article and with the pose from the images was measured (8.3, second and third part). Still to do: the tilt of the cereal boxes and the failures at level 3.

### Stereo pair and observation poses (8.1)

**Cameras.** With the simulation argument `stereo:=true` two cameras are added in the housing of the wrist camera, 25 mm either side of its centre (baseline 50 mm), with the intrinsics of the RGB-D camera (320 x 240, field of view 1.2 rad, fx = 233.9 px) and a little image noise (standard deviation 0.01, assumed, not calibrated). The left one is an `rgbd_camera`, so it also gives the depth of the simulator in its own frame, the reference to measure the stereo depth against (it has a noise of its own, see the accuracy below); the right one is a plain camera. The images are rectified by construction (pinhole, no distortion). Topics: `/stereo/left/{image,depth_image,camera_info}`, `/stereo/right/{image,camera_info}`; frames `stereo_left_optical_frame` and `stereo_right_optical_frame`. The RGB-D camera of step 4 is unchanged, apart from its mounting: it is rolled by 180 degrees about its optical axis (`rpy` of `camera_joint`), because its up axis is the x axis of the tool with a sign that left the images upside down when the tool is horizontal with its x axis up, the pose used to look at a shelf (the camera sits on top of the gripper). Checked by computing the image axes in that pose: before the roll the image down axis pointed up in the world, now it points down; and on the images of the simulation.

**Depth.** `amr_manipulator_perception/stereo_depth_node.py` matches the two images with OpenCV's semi-global block matching (128 disparities, so the nearest depth is 0.09 m, below the 0.1 m near clip; blocks of 5 pixels; a border on the left of both images, otherwise the first 128 columns have no depth) and publishes `/stereo/depth` (32FC1, NaN where there is no match) in the left frame. **It works on demand**: it keeps the last pair and computes when `/stereo_depth/compute` (a `Trigger` service) is called, or at a low rate with the parameter `rate`.

Real time factor, simulation with the warehouse and no GUI (four readings each, 4 s apart):

| Configuration | Real time factor |
|---|---|
| No stereo | 0.98 - 1.01 |
| Two more cameras | 0.86 - 1.03 |
| Cameras and node on demand (idle) | 0.93 - 1.00 (0.75 in the first reading, right after the start) |
| Cameras and node computing at the rate of the cameras (10 Hz) | **0.29 - 0.35** |

The matching at 10 Hz takes so much CPU that the simulation falls to a third of real time: hence the on-demand node.

**Accuracy**, measured by `scripts/stereo_check.py` against the depth of the left camera, pixel by pixel, 5 frames. **That reference is noisy**: the noise of the camera (standard deviation 0.01) is applied to the depth in metres too, and the residual of a plane fit on flat patches of a depth image is 10.1 mm (median of 38 patches, measured later, in 8.3). The errors below include it, so they are upper bounds of the error of the stereo depth. Robot at the station of shelf S3 (driven there, 0.25 m from the aisle), camera 0.38 m from the soup can of slot `S3.L2.n4`. The pixels are split by texture (local deviation of the grey image above 6 grey levels, the noise being about 2.5):

| True depth, pixels with texture | Pixels with a stereo depth | Bias [mm] | Median / 95th percentile of the absolute error [mm] |
|---|---|---|---|
| 0.30 - 0.50 m | 98 % | +0.1 | 8.3 / 31.6 |
| 0.50 - 1.00 m | 88 % | -66.2 | 21.2 / 557 |

Below 1 m, stereo gives a depth on 86 % of the textured pixels and on 46 % of the others, where it is wrong by tens of millimetres (median 34 mm between 0.3 and 0.5 m). In the working range the median error, 8.3 mm, matches what the geometry gives: at 0.4 m a disparity error of 0.5 pixel is 6.8 mm (depth error = depth^2 x disparity error / (fx x baseline)). The bias changed from +0.1 to +8 mm between runs and is not calibrated.

**Observation poses.** `moveit_check.py look --slot S3.L2.n4 --dist 0.30` puts the TCP 0.30 m in front of the centre of the article, tool horizontal into the shelf: IK seeded near that posture (a few tries each; the solver restarts from random states, so a solution can come with a joint turned by a multiple of 2 pi, and every solution is brought to the equivalent angles closest to 0 that the joint limits allow; among those with the base joint within +-1.6 rad, `PAN_LIMIT`, the one with the smallest largest joint is taken, and among all of them only if there is none: the shoulder turned the other way round was chosen in 4 to 18 of 35 slots, see 7.4) and a MoveIt joint goal. A first version rejected every joint beyond +-3.3 rad instead of normalizing it, and failed from the station with the wrist 2 at -4.2 rad, the same posture as +2.1. The camera is 0.08 m behind the TCP and looks along the tool z axis: with `--dist 0.30` it is 0.38 m from the centre of the article. From the station of S3 the arm went from the travel pose to the look pose of slot `S3.L2.n4` in 7.9 s (10 collision-free solutions found, the chosen one with a largest joint of 2.14 rad). Reach of the observation pose, bow towards the shelf, gap 0.25 m, all 105 pickable slots, with `reach_check.py --grasps look`:

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

**A camera without the robot.** The robot is what makes the generator slow: the navigation, the planning and 8 s to move the arm for every look pose. The images do not need the robot, only a camera with the same sensors: `models/dataset_camera` is a free camera (a kinematic model, no gravity) with the left camera and its true depth, the right camera and the segmentation camera, with the same topics and the same intrinsics as the robot; `launch/dataset.launch.py` starts the warehouse, that camera and the bridge, nothing else (no robot, controllers, Nav2 or MoveIt), and `stocker.set_pose` moves the camera with a service of the world. The gripper is not modelled on the free camera. By the URDF the camera is 0.08 m behind the tool point and the fingers end 0.01 m beyond it, so the fingertips are 0.09 m from the camera along its axis, closer than the near clip of 0.1 m, and the fingers are not rendered; yet every image of the robot has a fixed light shape in the bottom-left corner, whose cause was not identified (see the test with the robot below). The images of the free camera do not have it.

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

The shape, whose cause was not identified, is in the bottom-left corner: rows 204 to 239 and columns 52 to 114 of the image (2.2 percent of it), the same in every image. The segmentation camera does not see it, so the labels mark the articles behind it (2.4 percent of the article pixels, 12 of the 36 images) while the image shows the shape: the network answers background there, which is right for the image, and the labels count it as an error. The brightness of the images is the same as in the dataset (mean 97-99 against 92). The weakest class is the soup can, whose body is white or grey against a grey background and whose mask is often partial. The network was not trained again for the shape. A first attempt at this test was made by mistake in the empty world (the default of `sim.launch.py`) and was discarded; the script now passes `world:=warehouse.sdf`.

### Pose of the articles (8.3, first part)

**Method.** `amr_manipulator_perception/pose/geometry.py` (numpy and scipy, no ROS). The pixels of the mask of an article that have a depth (the mask is eroded by 2 pixels: at its edge the matching mixes the article with the background) are back-projected with the intrinsics of the camera; the points whose depth is far from the median (3 times the median absolute deviation, at least 4 mm) are dropped; the rest is taken to the map with the pose of the camera. Seen from above an article is a circle of known radius (a cylinder: x and y) or a rectangle of known sides (a box: x, y and yaw; the sides lie along the map axes at yaw 0 and the stock turns them by 15 degrees at most). The height is not estimated: the article stands on a shelf whose height is in the semantic map. The shape is fitted by robust least squares (soft L1 with a scale of 4 mm), and the points are compared only with the faces turned to the camera: otherwise the same points fit a rectangle in front of them or behind them and the depth is not determined.

**Checks without the simulation** (`pose/check_geometry.py`): the frames and the quaternion round trip, the back-projection, the rejection of the wrong depths, and the fit on points made up on the visible faces with a depth noise of 3 and 8 mm along the ray (centre error, 95th percentile: circle 0.7 and 2.7 mm, rectangle 2.5 and 4.2 mm). Then the code of the real chain on exact images made by tracing a ray through every pixel of a box or a cylinder seen from poses like those of the dataset: the centre error of the cylinders is 0.0 mm and that of the boxes 0.5 mm (median) and 4.1 mm (95th percentile), the yaw error 0.0 degrees. On exact images the fit adds no error.

**Measure in the simulation** (`scripts/pose_eval.sh`, `pose_report.py`). The world and the free camera of the dataset, with the stereo node; the shelves of a row are filled at random and the camera looks at them from poses like those of the arm. For every article of the picking face whose centre is in the image, the mask of its pixels is found by projecting its true centre (only the evaluation does this), and the pose is estimated and compared with the true one. 30 arrangements, 150 views, 232 articles; an estimate is made for 222 of them (96 percent: the others have less than 120 pixels in the mask or less than 30 points with a depth). Three estimates tell where the error comes from:

| Depth, mask | Centre error, median / 95th percentile [mm] | Along the row, median / 95th [mm] | Along the view, median / 95th of the absolute value [mm] |
|---|---|---|---|
| A: the depth of the simulator (noise 10 mm), true mask | 1.9 / 8.7 | 1.3 / 4.8 | 1.1 / 5.5 |
| B: stereo depth, true mask | 5.3 / 38.0 | 2.4 / 8.0 | 3.0 / 37.5 |
| C: stereo depth, mask of the network | 5.3 / 38.0 | 2.5 / 8.0 | 3.0 / 37.5 |

The network adds nothing: its masks differ from the true ones only at the edge pixels that the erosion removes. The error of the stereo is in the direction of the view, with a long tail (the worst, 129 mm, is a cereal box); along the row, the direction that matters for a grasp that closes along it, the 95th percentile is 8.0 mm. The tail comes from articles whose mask has no stereo depth on a large part (plain colours, no texture): when less than 60 percent of the pixels of the mask have a depth, 36 to 50 percent of the estimates are more than 20 mm wrong in depth, against 4 to 5 percent when 60 percent or more have one. With that fraction as a criterion of reliability, 180 of the 222 estimates (81 percent) are kept and the errors are:

| | Centre, median / 95th [mm] | Along the row, median / 95th [mm] | Along the view, median / 95th [mm] |
|---|---|---|---|
| all the estimates | 5.3 / 38.0 | 2.5 / 8.0 | 3.0 / 37.5 |
| 60 percent or more of the mask with a depth | 5.1 / 15.0 | 2.3 / 7.4 | 2.8 / 15.0 |
| 70 percent or more (140 estimates) | 4.8 / 12.9 | 2.0 / 6.7 | 2.7 / 12.9 |

The cylinders are estimated nearer than they are: the median depth error is 7 mm for the water bottle and the jam jar (4 mm for the soup can), against about 1 mm for the boxes; the cause was not investigated. The yaw of a box, with the stereo depth, has a median error of 4.3 degrees and a 95th percentile of 20 degrees: it was taken as not needed to grasp, because the widest box (70 x 50 mm turned by 15 degrees) is 80.6 mm across along the row, below the opening of 85 mm; that was wrong, see the cause of the tilt of the cereal box in the paragraphs of the grasp from the images below. The yaw is erratic with the depth of the simulator (median error 12.8 degrees), which has 10 mm of noise and no smoothing; this was first taken for a depth read as a distance along the ray (reading it so made the error larger, a bias of 28 mm), and was found by plotting the points seen from above.

**On the camera of the robot** (`scripts/pose_eval_robot.sh`). The same measure with the arm: Nav2 takes the robot to the station, MoveIt puts the arm in a look pose, the shelves are filled at random (4 shelf units, 4 look poses each, 4 fillings per pose). The pose of the camera is the one a grasp would have: the transforms from `base_footprint` to the left optical frame, from the joints, and the pose of the robot in the world from the simulator, not from Nav2 (its localization is off by centimetres, and what a grasp needs is the pose of the article relative to the robot). 64 views, 68 articles of the picking face in the image, an estimate for 59 (87 percent, against 96 percent with the free camera: the soup can 69 percent, and 61 percent of the articles between 0.48 and 0.58 m from the camera). Mask of the network, all the estimates: centre error median 6.1 mm and 95th percentile 36.8 mm; along the row 2.0 and 6.8 mm; along the view a median of 3.8 mm and a 95th percentile of 36.1 mm. With 60 percent or more of the mask with a stereo depth (48 estimates, 81 percent, as in the free camera): centre 4.8 and 14.9 mm (maximum 19.5), along the row 2.0 and 6.2 mm, along the view 3.7 and 14.8 mm. The cylinders are again estimated nearer than they are (the jam jar by 10.8 mm, the soup can by 6.0, the water bottle by 3.5). The mask of the network and the true mask give the same figures, although the images of the robot show that shape in the corner. The sample is small and the figures are an indication.

### Grasp with the true pose (8.3, second part)

`scripts/pick_oracle.py` (started by `pick_oracle.sh`, the series by `pick_series.sh`, the counts by `pick_report.py`) picks an article with its true pose, to test the arm and the gripper without the vision. The front grasp chosen in 7.4: pre-grasp pose 0.10 m before the centre of the article (the joints from the IK that the look poses use, seeded near the right posture and normalised, and a **joint goal**), a straight line to the centre, the gripper closes (25 N, one second of wait for the force to ramp), a straight line back 0.10 m and 0.02 m up. One second later the simulator says where the article is: the gripper state, the distance of the article from the TCP, the lift, and which neighbours moved. An attempt counts as *held* when the gripper reports GRASP and the article is within 40 mm of the TCP, *clean* when it is within 10 mm. The robot is spawned at the station of the unit (S1), with no Nav2.

What was wrong, in the order it was found (each one measured in the simulation):

- **A goal of pose lets MoveIt choose any IK branch.** The planned paths wound the elbow by 3.7 rad and wrist 1 by 5.8 rad in a trajectory of 10 s, the arm lagged by up to 4 rad and jammed; with the joint goal of `look_solution` the pre-grasp is reached.
- **The articles are static.** `stocker.create` makes them static (`<static>true</static>`, so that the stock stays still on the shelves): the fingers closed on an immovable object and the retreat pulled the robot. The article to pick is created again as a dynamic body where it stands (`static=False`).
- **The base does not stay at the station.** The wheels do not hold it: at the first movement of the arm after the spawn it was found 0.207 m and 46.5 degrees away from the station (0.208 m the next time), and an arm that pushes a static object slides it by up to a metre. The robot is put back with the `set_pose` service before every attempt, checked on its pose in the simulator, and the transform map to odom is set from that pose, so that the shelves of the planning scene are where they are.
- **The articles were not in the planning scene.** The arm hit the neighbours: 1 attempt held of 25 (the arm jammed, the base slid, every later pre-grasp failed). The neighbours are now boxes and cylinders in the scene, 4 mm larger than they are, except the one to pick.
- The gripper reports GRASP before the force has ramped to its value (`grip_ramp` 0.25 s): the retreat now waits 0.8 s, and a movement is judged by the TCP error measured after it (less than 8 mm), not by the answer of the action server, which refuses an arm that stopped 0.0377 rad from its goal.

With one article alone on the row (S1, levels 1 and 2): 5 attempts of 6 held, the article 0.3 to 3.6 mm from the TCP and 19 to 20 mm up, the neighbours not touched, the base still. With the neighbours, in a series that goes on in the same world, an arm that jams spoils all the following attempts (4 held of 16, then eight failures in a row). **A new world for every attempt, 30 attempts at the unit S1, the neighbours present:**

| | Attempts | Held | Clean (within 10 mm) |
|---|---|---|---|
| all | 30 | 19 (63 percent) | 10 |
| level 1 | 9 | 8 | 4 |
| level 2 | 8 | 5 | 4 |
| level 3 | 13 | 6 | 2 |
| cereal box | 8 | 4 | 0 |
| juice carton | 5 | 3 | 0 |
| soup can | 6 | 5 | 4 |
| water bottle | 4 | 2 | 1 |
| parcel | 3 | 2 | 2 |
| jam jar | 4 | 3 | 3 |

The 11 attempts not held: 6 *lost* (the gripper held the article during the closing, but after the retreat it was 41 to 48 mm from the TCP and lifted by 27 to 31 mm, not 20: it slipped or tilted in the fingers, see below; the tall articles, a cereal box in 3 of the 6), 2 closed on nothing (one with the article more than a metre from the TCP), 2 approaches in which the arm did not follow (the TCP 0.9 and 1.4 m from the target, both at the slots at the ends of level 3), and 1 pre-grasp for which the planner returned an invalid path four times. Only the lost grasps were looked into, below; the others were not diagnosed. The cereal boxes and the juice cartons, the two tall boxes (0.16 and 0.17 m), are never held within 10 mm.

**The tall articles are held, but they tilt during the retreat.** The distance of the article from the TCP in the series above was computed as if the article stood upright. Measured again with the position and the orientation of the article read from the simulator, 12 attempts with one tall article alone on the row, levels 1 and 2 (juice carton, cereal box, water bottle): in all 12 the article was upright (0.0 degrees) before and after the closing, centred on the TCP within 0.5 mm, with forces between 24.5 and 25.5 N; after the retreat all 12 were still with the gripper, within 6 mm of the TCP, and tilted: the juice cartons by 0.3 to 0.4 degrees (3 of 3), the cereal boxes by 2.4, 14.7, 35.6 and 35.9, the water bottles by 13.3 to 26.3 (5 of 5). So the "lost" grasps of the series are probably held and tilted articles (not verified on those data, which have no orientation). With a lift of 3 cm straight up before the retreat (the same articles in the same slots): the cereal boxes 0.3 and 8.4 degrees (2.4 and 14.7 before), the bottles 10.6 and 21.4 (18.6 and 25.6), the cartons 0.1 and 0.2; one bottle, 26.3 degrees before, fell off the shelf (90 degrees, 270 mm lower), and the pre-grasps that followed failed, so that run was stopped after 7 attempts. The tilt starts in the retreat, not in the closing. **Gripping above the centre of mass.** An article held by its centre of mass is in neutral equilibrium about the axis between the pads, and any disturbance of the retreat turns it; one held above hangs under the axis like a pendulum. The test, 10 bottles in 10 fresh worlds with the grasp point at the centre of the article, and the same 10 slots with the grasp point 4 cm above it (`--grasp-dz 0.04`): in the six slots at the centre of the unit, held in both series, the tilt after the retreat has a median of 13.1 degrees (maximum 17.0) with the grasp at the centre and 0.9 degrees (maximum 2.9) with the grasp 4 cm above it. At the end slot n5 (4 of the 10) it is the opposite: with the grasp at the centre all 4 were held (tilt 10.6 to 27.8 degrees); with the grasp 4 cm above none: one bottle fell (180 degrees, 120 mm lower), one closing was on nothing, in one the bottle stayed in place and the arm ended 1.1 m from it, and one approach failed. The cause at the end slots was not diagnosed (a guess: the reach of the arm at the end of the unit, where the failures of level 3 are too). **Boxes do not gain from the raised grasp.** The same test, 10 + 10 attempts in fresh worlds, juice cartons and cereal boxes, levels 1 and 2, the slots at the centre of the unit only, all 20 held: with the grasp at the centre the tilt after the retreat is 0.3 to 0.4 degrees for the 5 juice cartons and 0.2, 0.2, 4.7, 27.2 and 35.9 for the 5 cereal boxes (the last two in the same slot, S1.L2.n2); with the grasp 4 cm above it is 0.2, 2.1, 7.5, 9.6 and 15.8 for the cartons and 0.2, 0.4, 7.5, 8.4 and 10.9 for the cereal boxes. The raised grasp helps the worst cases of the cereal box and makes the cartons worse, so what it fixes is the bottle, a cylinder, and the height of the grasp point would have to depend on the class. **Cylinders.** The soup can and the jam jar (0.10 and 0.075 m high) with the grasp point moved by 22 percent of the height, as for the bottle (0.02 and 0.015 m): 6 attempts at the centre and the same 6 slots raised, fresh worlds, slots at the centre of the unit, levels 1 and 2. The soup can is held 6 of 6 both ways, with a tilt after the retreat of median 9.8 degrees (maximum 23.1) at the centre and 1.6 (maximum 2.9) raised. The jam jar is held 3 of 6 at the centre (the other three approaches failed, in slots where the raised grasp worked), with tilts of 11.7, 0.4 and 11.9 degrees, and 6 of 6 raised, with a median of 2.4 (maximum 8.5). The three failed approaches at the centre may come from the body of the gripper, 60 mm thick, with the TCP 37.5 mm above the board; not verified. The height of the grasp point is therefore a property of the class: `grasp_dz` in `articles.yaml` (bottle 0.04, soup can 0.02, jam jar 0.015, cereal box and juice carton 0, the parcel was not tried and is 0), and `pick_oracle.py` uses it unless `--grasp-dz` is given.

**The series again, with the height of the grasp per class and the tilt measured.** The same 30 draws as the first series (unit S1, the neighbours present on the row and in the planning scene, all levels, all slots, a fresh world for every attempt), the article read from the simulator with its orientation: **24 attempts of 30 held (80 percent)**, all 24 within 10 mm of the intended position of the article with respect to the TCP. By level: 7 of 9 at level 1, 8 of 8 at level 2, 9 of 13 at level 3. By class: cereal box 8 of 8, water bottle 4 of 4, soup can 4 of 6, jam jar 3 of 4, juice carton 3 of 5, parcel 2 of 3. Tilt of the 24 after the retreat: median 5.2 degrees, maximum 35.2; 11 below 5 degrees, 6 between 5 and 20, 7 above 20. The cylinders and the parcel are good (water bottle at most 3.3 degrees, soup can 5.4, jam jar 8.7, parcel 0.3); the cereal boxes are the problem: 6 of 8 tilted by 28.5 to 35.2 degrees, five of them by 33.7 to 35.2; the three juice cartons by 14.4, 16.8 and 26.4. How much the article was turned (the closed width over its nominal width) does not explain it: cereal boxes with a width 1 to 6 mm over the nominal were upright and tilted. The 6 not held: 2 pre-grasps (the planner returned an invalid path four times), 3 approaches in which the arm did not follow and 1 closing on nothing with the article 1.28 m away, all four at level 3 (the soup can at n5, the juice carton at n4, the parcel and the jam jar at n1). Slots at the ends of the unit (n1 and n5): 10 of 14 held; central slots: 14 of 16. The cause of the tilt of the cereal box was not diagnosed; the failures at level 3 were, see below.

**Level 3: the branch of the pre-grasp.** A series of 12 attempts at level 3 only (water bottle, one article on the shelf, slots n1, n2, n3 and n5, the same draws on every repetition) held 9; the three not held were all at n2 (twice) and n3, and an earlier series of 10 had held 5. The plan of the straight approach and the joints of the pre-grasp were recorded for every attempt. The nine held had the pre-grasp with the elbow at -1.10 rad and an approach of 19 points, 1.8 s, with the largest joint change of 0.47 rad. The three not held had it with the elbow at +0.67 or +0.84 rad and wrist 1 at +0.7: the same position and orientation of the gripper, reached on another branch of the inverse kinematics, from where the straight line into the shelf needs 140, 171 and 191 points, 14 to 19 s and a joint change of 4.7, 6.4 and 7.4 rad (each wrist joint turned by half a turn or more). The arm lags behind the reference, the torque limit is reached, the bottle is knocked or the closing finds nothing. `look_solution` returned that branch because it takes the solution with the smallest largest joint among a few restarts of the IK, and at the slots whose pre-grasp is far from the base the other branch is not always found, or is the larger one. The fix (`--best-pregrasp` in `pick_oracle.py`, `look_candidates` in `moveit_check.py`): collect all the solutions (3 to 8 per attempt; in 22 of the 30 attempts of the series below at least one of them had an approach of more than 40 points), add seeds near the postures that gave the short approach, compute the straight line from each (`compute_cartesian_path` from the joints of the candidate) with the article taken out of the planning scene, as it is in the approach, keep those that complete and order them by the sum of the joint changes; the planner is asked for the best, and for the next one if it fails (three at most). The first version judged the lines with the article still in the scene (three attempts found no complete line: 54 to 64 percent) and had no next candidate (one attempt more), and gave 25 of 30; with both corrected, **level 3 only: 11 of 12 before the extra seeds, and the failing slot n3 alone held; the whole series: 29 of 30 held (97 percent), all 29 within 10 mm of the intended position**, against 24 of 30 for the same 30 draws without it. Level 1: 8 of 9, level 2: 8 of 8, level 3: 13 of 13; ends of the unit (n1, n5) 13 of 14. By class: cereal box 8 of 8, water bottle 4 of 4, soup can 6 of 6, jam jar 4 of 4, parcel 3 of 3, juice carton 4 of 5. Of the 30, five that were not held before are held (three approaches in which the arm did not follow, one closing on nothing, one pre-grasp), none went the other way. The one not held is the juice carton at L1.n5: the planner returned an invalid path for all the three candidates, four times each. Tilt of the 29 held: median 3.8 degrees; 16 below 5, 2 between 5 and 20, 11 above 20 (eight cereal boxes at 22.9 to 35.6, three juice cartons at 26.8 to 29.8); the tilt did not change. One unit and 30 draws: one attempt more or less is within what the planner does by chance.

**Level 1: the swing to the pre-grasp.** `--best-pregrasp` ranks the solutions of the pre-grasp by the joint travel of the straight line only. At level 1 several solutions have a line of about 0.7 rad, and the choice among them is not about the line: in the Nav2 series with the shift (second one) the chosen pre-grasps of 4 attempts had the base joint at 2.7 to 3.0 rad, 3 to 4 rad away from the joints of the look pose they started from, and each of them needed at least one planner failure (-2) before the arm got there; the cereal box at L1.n1 that was not held was one of them (-2, -2, -4, then reached, lying on its side before the closing). `--swing-aware` takes, among the solutions whose line moves the joints at most 1 rad more than the best, the one nearest to the joints of the arm now. Paired test at level 1 (12 draws, seed 3, one article on the shelves, images with the robot put back by teleport, a fresh world for every attempt; `/tmp` of the container): the sum of the joint changes from the look pose to the pre-grasp, median **10.5 rad** with the ranking by the line and **1.5 rad** with `--swing-aware`; attempts that needed a planner retry (-2 or -4) 6 of 11 and 1 of 10; planner failures per attempt with a plan 1.73 and 0.10. The attempts held are the same number, 10 of 12 both ways, with other failures: bottle at L1.n5 with 12 failed tries (baseline) and bottle at L1.n3 with no solution with a complete line (swing-aware, the same slot was held in the repeat of the draw); the jam jar at L1.n3 in both has no collision-free IK for any of the three look poses (not looked into). 12 attempts cannot tell a difference of a few attempts in the success rate: what is measured is the planner trouble, not the success. The Nav2 series were made without it; the attempt 6 of the first one (a normal posture, -4 eleven times) is not explained by it. Not yet the default: it changes the choice at levels 2 and 3 too, and needs the 30-series again.

**The tilt of the cereal box in time (`--trace-tilt`, `tilt_trace.py`, `tilt_report.py`).** `--trace-tilt` records the pose of every entity of the world (one message every 20 ms of simulated time) from the closing to the end of the retreat; the report gives the tilt of the article, and its lean along the approach and across it, against the start of the retreat. 16 cereal boxes alone on the shelves, true pose, all levels (seed 6): 13 below 5 degrees and 3 above (6.7, 16.9 and 36.1), all three at level 3 (3 of 7 there, 0 of 9 at levels 1 and 2). In the three, the lean is along the approach (about the axis between the pads), the centre of the box stays within 2.7 to 7.5 mm of the TCP (it turns about itself) and the angle appears as a jump: in two within 0.4 s from the start of the retreat, in the third (36 degrees) 0.3 s after the arm had stopped. Once the arm is at rest the angle does not change (it did in 1 of 20 traces examined): the box is not swinging, it is held at the angle where the transient left it. **What was tried and does not explain it.** (1) *Grasp above the centre* (`--grasp-dz` 0, 0.01, 0.02, 0.03; 10 cereal boxes alone at level 3 each, seed 7, the same draws, all 40 held): above 5 degrees 1, 5, 4 and 3 of 10 (medians 0.6, 9.4, 4.2 and 0.7 degrees; maxima 11.0, 24.7, 14.9 and 16.3): raising the grasp makes the tilt more frequent, not less, as it does for the cylinders. (2) *An error of centring along the approach* (a mass not at the axis between the pads gives a torque): with the true pose the article is centred within 0.1 to 0.4 mm in every attempt; in the second Nav2 series the cereal boxes tilted by 32 to 36 degrees had an error in depth of 0.1 to 0.3 mm, those upright of up to 3.6 mm (correlation between the size of the error and the tilt -0.65, the wrong sign). (3) *The posture of the pre-grasp*: no pattern (the postures with the wrist 3 at +1.57 rad, used at levels 1 and 2, tilted 1 time in 9; the others are level 3 almost all; and the same slot with the same joints gave 0.4 to 0.7 degrees five times with the grasp at the centre and 17 to 25 degrees with 1 cm above it). What differs between the series is something else: the true pose with neighbours on the row (the first series with the grasp height per class) tilted 6 of 8 cereal boxes by 28 to 35 degrees, the second Nav2 series 5 of 7 by 14 to 36, and the cereal boxes alone 4 of 26 (all at level 3). Whether it is the neighbours (in the planning scene and on the shelf), the images or the `--best-pregrasp` is not known: the cause of the tilt is **not diagnosed**.

**The cause: the yaw of the box (found).** The stock turns every box by a random yaw of up to 15 degrees (`yaw_jitter_deg`) and the grasp, which comes straight into the shelf, ignores it. The closed width of the gripper shows how the box was held: 70.1 to 70.2 mm (its width) with the box not turned, up to 79.3 mm with it turned (the pads on two corners). In the four Nav2 series 18 of the 19 cereal boxes held were tilted by 28.5 to 37.7 degrees, and every box with a closed width of 75.9 mm or more (a yaw of about 7 degrees or more) was. Three conditions at the same slot (S1.L2.n2, cereal box, true pose, fresh worlds, 6 attempts each, the draws of the stock the same in A and C): A neighbours on the shelves and in the planning scene, 2 of 5 tilted (one attempt gave no result); B the box alone and not turned, 1 of 6 (16.8 degrees, the closed width 70.2 mm); C neighbours on the shelves but not in the planning scene (`--no-scene-neighbours`), 4 of 6. So the planning scene is not what tilts the box; what the neighbours change is that the boxes are turned. At the same yaw A and C differ in 2 of 5 pairs (1.1 and 0.5 degrees in A against 35.7 and 33.9 in C), so below about 4 degrees of yaw the outcome is not set by the yaw alone. The test of the yaw (the box alone, true pose, 4 attempts for each condition): turned by +12 degrees and approached straight, **4 of 4 tilted, 32.0 to 32.2 degrees**, closed width 77.3 to 77.4 mm; turned by -12, **4 of 4 tilted, 31.8 to 31.9 degrees** (77.4 to 77.6 mm); approached turned by the yaw of the box (`--align-yaw`, the approach direction rotated by the true yaw, so that the pads close on two faces), **8 of 8 upright, 0.7 to 2.0 degrees**, closed width 70.1 to 70.3 mm, all held. The note in 8.3 that the yaw of a box is not needed to grasp (the widest box, turned by 15 degrees, is 80.6 mm across, below the opening of 85 mm) was wrong: the box is held, but on its corners and it leans. What is not done: the yaw used in these tests is the true one, and the yaw estimated from the images has a median error of 4.3 degrees and a 95th percentile of 20 (12.8 median on the depth of the simulator, 8.3); the 16.8 degrees of the box not turned (B) and the box tilted at a closed width of 70.5 mm are not explained. **The tolerance to an error of the yaw (`--yaw-error-deg`)**: the box alone at S1.L2.n2 turned by 12 degrees, approached turned by the true yaw plus an error, 3 attempts for each error, fresh worlds. Error +3 degrees: 2 of 3 upright (0.5 and 0.7), one tilted by 34.9; -3: 1 of 3 upright, two tilted by 34.9 and 35.0; +6: 2 upright (0.6, 0.7), one tilted by 33.0; -6: all 3 tilted by 34.0 to 34.1; +9: one upright, two closings on nothing (the box 221 mm from the TCP and lying at 90 degrees: the approach knocked it over); -9: all 3 tilted by 35.2 to 36.1. With the exact yaw 8 of 8 were upright; with errors of 3 and 6 degrees 5 of 12 were (7 of 12 tilted by 33.0 to 35.0); at 9 degrees 1 of 6, at 12 degrees (straight approach) 0 of 8. So the grasp does not tolerate even 3 degrees of error, and an estimate of the yaw from the images with a median error of 4.3 degrees and a 95th percentile of 20 cannot fix the tilt as it is. **A weaker grip and a slower retreat do not help**: the box alone turned by 12 degrees, approached straight (32.0 to 32.2 degrees with the defaults, 4 of 4), 3 attempts each: grip force 8 N, tilted by 33.3, 33.3 and 34.0 degrees; 15 N, 32.6, 32.1 and 32.1; retreat at a speed scale of 0.1 instead of 0.3, 31.5, 32.1 and 31.6; 8 N and 0.1, 32.3, 32.7 and 32.5. All 12 held and tilted, the closed width 77.3 to 77.5 mm in all of them: the box does not turn between the pads at any of these forces. The closing itself (the controller's approach force of 12 N, the stall detection, the ramp to the grip force) was not changed, as it is in the C++ of the gripper controller. Not tried: that, and the yaw from the images used in the grasp.

**Level 1 again, with the gap of 0.30 m, the look pose within +-1.6 rad and `--swing-aware`** (the same 12 draws, seed 3, `/tmp` of the container): **11 of 12 held**, 9 within 10 mm (the two jam jars are at 15.0 and 14.9 mm). No pre-grasp has the base joint beyond 1.6 rad (0 of 12, against 5 of 11 with the ranking by the line and 3 of 10 with `--swing-aware` alone at 0.25 m), the swing from the look pose to the pre-grasp has a median of 1.8 rad, and the 11 attempts that had a plan reached the pre-grasp at the first try. The jam jar at L1.n3, without a collision-free look pose at 0.25 m in both earlier runs, is held. The one not held is the water bottle at L1.n5 (the end of the unit), as in the first run: the solutions in the normal posture have a straight line of 189 to 195 points (21 to 26 rad of joint travel) and the only short line (15 points) is the one with the base joint at -2.1 rad, 12 rad away from the look pose; the planner failed ten of twelve tries with -2 and two with -4. **The look pose near the branch of the pre-grasp.** In `see()` (with `--best-pregrasp`) the pre-grasp of the nominal slot is found first (`branch_pregrasp`: among the solutions with a complete line within 1 rad of the best, the one with the base joint within +-1.6 rad, the best line if there is none), the look pose is the candidate nearest to it in the sum of the joint changes, and the comfortable look pose within +-1.6 rad is the second try if it cannot be reached. The same 12 draws: **12 of 12 held**, 10 within 10 mm (the two jam jars at 15.0 mm); no attempt needed a planner retry (0 of 12; 6 of 11, 1 of 10 and 1 of 12 in the three runs above), the swing from the look pose to the pre-grasp has a median of 1.6 rad, and one pre-grasp is turned over (the water bottle at L1.n5: its branch, with the base joint at -2.1 rad, is reached at the first try from a look pose in the same posture). One series of 12 draws at one level: it shows that the failure of the end slot is gone, not a rate.

### Grasp with the pose from the images (8.3, third part)

`pick_oracle.sh` with `VISION=1` (`--vision`, `scripts/vision_target.py`): the pose of the article is no longer the true one. The arm goes to a look pose in front of the **nominal** slot (0.30 m of TCP distance, then 0.22 and 0.38 if the estimate is not reliable: the pose of the slot is known from the semantic map, the article may be a centimetre off and turned), the left image goes to the network, the pixels of its mask nearest to where the slot projects (the class of the article is the one the task asks for) are back-projected with the stereo depth, and the shape of the class is fitted (8.3, first part). In the first version an estimate with a depth on less than 60 percent of the mask was not used (replaced, see below). The estimated centre replaces the true one in the pre-grasp and in the approach; the height is the shelf's and the class's. The camera is taken to the world with the transforms of the arm and the pose of the robot from the simulator, and the estimate is taken back to the robot with the same pose, so that the localization does not enter. The same 30 draws as the series with the true pose (unit S1, neighbours present, fresh world for every attempt):

| | True pose | Pose from the images, first version |
|---|---|---|
| held | 24 of 30 | **21 of 30 (70 percent)** |
| within 10 mm of the intended position | 24 | 19 |
| level 1 / 2 / 3 | 7 of 9 / 8 of 8 / 9 of 13 | 7 of 9 / 4 of 8 / 10 of 13 |
| cereal box / bottle / juice carton | 8 of 8 / 4 of 4 / 3 of 5 | 7 of 8 / 4 of 4 / 3 of 5 |
| soup can / jam jar / parcel | 4 of 6 / 3 of 4 / 2 of 3 | 2 of 6 / 3 of 4 / 2 of 3 |

The error of the 26 estimates that were made, against the true pose: across the row (the direction the fingers close) median 1.5 mm, 95th percentile 4.3, maximum 5.8; along the view median 2.0 mm, 95th percentile 5.6, maximum 18.6. The two jam jars that were held at 15.0 mm instead of within 10 mm are the cylinders that the stereo gives nearer than they are (3.4 in 8.3). The 9 not held: **4 without an estimate, all soup cans**, with a depth on only 44 to 57 percent of the mask at the three look distances (the body is white and plain: no texture for the stereo), while two other soup cans were estimated and held (so the 60 percent limit, which was taken from the evaluation of 8.3, leaves this class on the edge); 2 pre-grasps in which the planner returned an invalid path four times (a cereal box and a juice carton, both at L2.n4); 2 approaches in which the arm did not follow and 1 closing on nothing, all three at level 3. Tilt of the 21 held: median 1.4 degrees; 13 below 5, 1 between 5 and 20, 7 above 20 (five cereal boxes at 25.2 to 35.3, two juice cartons at 26.1 and 29.6), as with the true pose.

**The limit on the mask, replaced.** The four soup cans without an estimate had a depth on 44 to 57 percent of the mask, and that fraction does not separate the good estimates from the wrong ones. With 172 soup cans estimated in the simulation of 8.3 (shelves with soup cans only), 15 estimates were wrong (more than 20 mm across the row or more than 40 mm along the view; 4.1 to 72 cm from their slot), in every band of the fraction, and with the depth of the simulator the 13 that were looked at in an earlier run of the same kind were accurate: the error is the stereo depth. What tells them apart is the distance from the nominal slot: the true article is within 1.4 cm of its slot, the estimates that were right were within 4.8 cm and the wrong ones from 4.1 cm. A limit of 3 cm to the nominal slot keeps 151 of the 172 soup can estimates (88 percent), none of the wrong ones, with an error across the row of 6.2 mm (95th percentile) and along the view of 11.9 mm; on all the classes (258 estimates, 36 arrangements) it keeps 239 (93 percent), none of the 10 wrong ones, with 7.7 and 13.6 mm, against 212 (82 percent) and one wrong left for the limit of 60 percent of the mask. It replaced it in `vision_target.py` (3 cm to the slot, and a depth on at least 30 percent of the mask). The same 30 draws again: **26 of 30 held (87 percent)**, 24 within 10 mm of the intended position. Level 1: 9 of 9, level 2: 8 of 8, level 3: 9 of 13. Cereal box 8 of 8, water bottle 4 of 4, soup can 5 of 6, juice carton 4 of 5, jam jar 3 of 4, parcel 2 of 3. Every attempt had an estimate at the first look (0.30 m of TCP distance) and none was rejected by the limit of 3 cm; the error of the 30 estimates is, across the row, a median of 1.2 mm, a 95th percentile of 4.0 and a maximum of 8.4, and along the view a median of 2.0, a 95th percentile of 7.2 and a maximum of 15.2. The four not held are all at level 3: a closing on nothing with the jam jar at n1, an approach in which the arm did not follow with the soup can at n5, the juice carton at n4 knocked over (90 degrees), and an approach with the parcel at n1. Tilt of the 26 held: median 3.0 degrees; 15 below 5, 2 between 5 and 20, 9 above 20 (the cereal boxes and the juice cartons). The 26 against the 24 of the series with the true pose is not an advantage of the images: that series had two pre-grasps in which the planner returned an invalid path four times, and this one none; the planner is random.

**With the choice of the branch of the pre-grasp.** The same 30 draws with `--best-pregrasp` (see the level 3 paragraph above): **30 of 30 held, 26 within 10 mm of the intended position** (the four above 10 mm: two jam jars at 15.0 mm, a soup can and a water bottle at 10.8 mm). Level 1: 9 of 9, level 2: 8 of 8, level 3: 13 of 13; every class held in all its attempts. Every attempt had an estimate at the first look and none was rejected. The error of the 30 estimates is, across the row, a median of 1.4 mm, a 95th percentile of 6.0 and a maximum of 7.7, and along the view a median of 2.3, a 95th percentile of 18.8 and a maximum of 19.7. Tilt of the 30: median 2.4 degrees; 18 below 5, 3 between 5 and 20, 9 above 20 (five cereal boxes at 21.3 to 35.6, four juice cartons at 22.3 to 30.0). The 30 against the 29 with the true pose is not an advantage of the images: the one attempt not held with the true pose is a planner failure (the pre-grasp), and the planner is random.

### Pick with Nav2 (8.3, fourth part)

Until here the robot was put back at its station by teleport before every attempt. `NAV=1 pick_oracle.sh` (`--nav`, with `--vision`) removes it: the robot spawns at the origin, `nav.launch.py` starts Nav2 with the saved map, `nav_semantic.py` drives to the station of the unit (goal tolerance 0.10 m and 0.10 rad), then the attempt goes on as before (arm to the look pose, network, stereo, pre-grasp, approach, closing, retreat). The pose of the robot is now the one **believed** by AMCL (the transform map to `base_footprint`): the look poses, the estimate and the planning scene are built with it, and the estimate is taken back to the robot with the same transform, so the error of the localization cancels in the pose of the article relative to the gripper. It enters in three places: the distance of the estimate from the nominal slot (the limit of 3 cm was for a perfect localization; with Nav2 it is 6 cm, `--prior-radius`), the search of the mask (in the first version the component of the class nearest to where the believed pose projects the slot, within 20 px) and the shelves of the planning scene. The judging (is the article in the gripper) always uses the pose of the simulator.

**Arrival.** In the 30 attempts of the series below the robot stopped 17 to 88 mm from the station (median 42 mm) and 0 to 8 degrees from its heading (median 2.9). The localization was worse than that: AMCL believed the robot 5 to 168 mm from where the simulator had it (median 40 mm). The error is almost all **along the shelf** (belief minus truth along x: mean +38 mm, standard deviation 40, largest 167; across, towards the shelf: mean 9 mm, standard deviation 8, largest 27): in front of a flat face of boards the LiDAR fixes the distance and the heading, not the position along it. Five no-motion updates (`/request_nomotion_update`, since AMCL does not update while it stands still) took the median from 77 to 40 mm.

**AMCL.** Three runs of the same draws, from the file installed with Nav2 to a tighter one (`config/nav2_params_precise.yaml`, chosen with `params:=` in `nav.launch.py`; `NAVPARAMS=precise`): 180 beams instead of 60, `sigma_hit` 0.05 instead of 0.2, `z_hit` 0.9 and `z_rand` 0.1. With the default file and the 3 cm limit, 30 attempts held 8 (21 without an estimate); with the default file, the 6 cm limit and the updates, 3 of 8; with the tight file, 7 of 8. The figures of step 6 (navigation) were measured with the default file, which is unchanged.

**Result of the first version (search near the projected slot), tight AMCL, 6 cm limit, `--best-pregrasp`, the same 30 draws, a fresh world for every attempt: 20 of 30 held (67 percent), 18 within 10 mm of the intended position** (30 of 30 with the robot put back by teleport). Level 1: 4 of 9, level 2: 5 of 8, level 3: 11 of 13. Cereal box 5 of 8, jam jar 3 of 4, juice carton 4 of 5, parcel 3 of 3, soup can 3 of 6, water bottle 2 of 4. Tilt of the 20 held: median 2.5 degrees; 13 below 5, 1 between 5 and 20, 6 above 20 (the cereal boxes and the juice cartons). The estimate, when there was one (23 attempts), has an error in the frame of the robot of a median of 1.6 mm across and 2.1 mm along the view (95th percentile 4.5 and 12.5; maximum 6.4 and 14.0): the same as with the teleport. The 10 not held:

- **7 without an estimate**, all from the localization: in 4 (cereal box L1.n1, soup can L1.n5, juice carton L1.n1, cereal box L2.n4) the belief is off by 82 to 168 mm along the shelf, the slot projects that far from the article and no pixel of the class is within 6 px of it; in 2 (soup cans L1.n2 and L3.n5, belief off by 57 and 61 mm) the estimate is there and consistent in two looks, 6.2 to 6.6 cm from the nominal slot, just beyond the limit of 6 cm; in 1 (cereal box L2.n4, belief off by 51 mm) the nearest cereal box was a neighbour, 49 to 52 cm from the slot. Of the 19 attempts with a belief off by 50 mm or less along the shelf none was without an estimate; of the 11 with more, 7 were and 4 were held (at 52, 53, 55 and 59 mm): the limit of this setup is between 50 and 80 mm.
- 1 pre-grasp with no complete approach (water bottle L3.n1).
- 2 closings on nothing, with the pre-grasp and the approach done: a water bottle (L1.n2) found tilted by 17 degrees before closing and then knocked over, and a jam jar (L2.n2) whose pre-grasp was 38 mm off across the row though the estimate was off by 1.5 mm; the cause of the second was not investigated.

**Finding the shift from the images (`--localize`, `scripts/localization.py`).** The 7 failures without an estimate came from asking the believed map where the article is. The shift along the shelf is observable from the image instead: `Vision.detect` takes every article in the image (the masks that touch the border are cut and are not used), each with its pose in the believed map; the inventory (the class in each slot of the face and level: here taken from the stock of the simulator, as a warehouse system would give it) says where each class should be. A shift of the believed map is a candidate when it puts a detection on a slot of its class (within 4.5 cm across the face, where the LiDAR is good, and 25 cm along it); the one that puts the most detections on slots of their class is taken (the mean of the residuals of those, then the nearest again), with the smallest shift if two tie, and with it the article of the slot to pick. If another shift explains as many articles with another article for the slot, nothing is chosen (the look is tried at the next distance). The part along the face is applied to the 14 static objects of the planning scene (read from move_group, moved from their original place) and to the neighbours; the grasp itself needs no shift, the estimate being relative to the robot. `localize_check.py` tries it without ROS on made-up shelves (5 slots 0.2 m apart, random classes, shifts of up to 20 cm, noise of 5 mm, the articles within 26 cm of the centre of the view): 3979 of 4000 right, 16 without a result, 5 wrong, all five flagged ambiguous (four or five articles of the same class in a row).

**Result with the shift, the same 30 draws, tight AMCL, `--best-pregrasp --localize`: 29 of 30 held (97 percent), 27 within 10 mm of the intended position** (the two above: jam jars at 12.6 and 14.8 mm), against 20 of 30 with the search near the projected slot and 30 of 30 with the robot put back by teleport. Level 1: 8 of 9, level 2: 8 of 8, level 3: 13 of 13. Cereal box 7 of 8, the other five classes all held. The shift was found in all 30 attempts, 29 at the first look (0.30 m) and one at 0.22 m; none was ambiguous. The shift along the face against the true one (the believed map against the simulator's, at the slot): the difference has a median of 2.8 mm, a 95th percentile of 9.2 and a maximum of 15.6 mm, while the true shift reached 130 mm. The robot stopped 4 to 91 mm from the station (median 32 mm) and 2.8 degrees (median, 7.9 at most) from its heading; the belief was off along the shelf by a median of 22 mm and at most 122 mm (7 attempts with 50 mm or more, 4 with 80 mm or more: 122, 117, 89 and 88 mm, all four held). The error of the estimates in the frame of the robot: across a median of 1.4 mm, a 95th percentile of 4.4 and a maximum of 5.8; along the view 2.5, 10.4 and 17.9 mm. Tilt of the 29 held: median 2.7 degrees; 18 below 5, 1 between 5 and 20, 10 above 20 (the cereal boxes and the juice cartons). The one not held, the cereal box at L1.n1, is not a localization failure: the shift was found (10.4 mm against a true 3.3), the belief was off by 14 mm, and the pre-grasp ended with the controller failing (code -4) in 11 of the 12 tries over the three candidates and -2 in the other. **What this does not show:** the arrival of Nav2 is not repeated by the seed, so the two series differ in the arrival as well as in the method (this one had a smaller error: a median of 61 mm before the five updates against 77, and 23 against 40 after, 7 attempts with 50 mm or more against 11); but the 11 of the first series had 7 attempts without an estimate, and the 7 of this one were all held. One series of 30 on one unit; the inventory is the simulator's; the neighbours of the scene are placed from the simulator, moved by the shift.

**A second series, new draws (seed 2, same setup): 27 of 30 held (90 percent), 25 within 10 mm.** The draws are not the 30 on which the method was put together, and the arrival of Nav2 is a new one. Level 1: 7 of 9, level 2: 8 of 8, level 3: 12 of 13. Cereal box 7 of 8, juice carton 7 of 8, water bottle 4 of 4, parcel 3 of 3, soup can 5 of 5, jam jar 1 of 2. The shift was found in all 30 attempts (28 at the first look, 2 at 0.22 m), none ambiguous; against the true one the difference along the face has a median of 3.7 mm, a 95th percentile of 10.2 and a maximum of 14.1 mm, with a true shift of up to 95 mm (across the face the true shift reached 28 mm, not corrected). Arrival: 23 to 102 mm from the station (median 38), 2.7 degrees (median); belief off along the shelf by a median of 25 mm and at most 89 mm (5 attempts with 50 mm or more, 1 with 80 mm or more), a median of 67 mm before the five updates and 29 after. Error of the estimates in the frame of the robot: across a median of 1.1 mm (95th percentile 4.5, maximum 5.0), along the view 1.7 (7.0, 24.2). Tilt of the 27 held: median 2.9 degrees; 16 below 5, 4 between 5 and 20, 7 above 20. The three not held are not localization failures (the shift was found within 9, 7 and 6 mm of the true one along the face): a cereal box at L1.n1 whose pre-grasp needed four tries (-2, -2, -4, then reached) and which lay on its side (90 degrees) before the closing, and two approaches (a juice carton at L1.n4 and a jam jar at L3.n2) in which the arm did not follow the line (code -4, TCP 13.4 and 12.6 mm from the goal), the kind of failure of the series with the true pose. Both series together: **56 of 60 held (93 percent), 52 within 10 mm**; the cereal box at L1.n1 was not held in either (in the first series the pre-grasp ended with the controller failing), and the one of the first series (the same draw) was held with the robot put back by teleport: a possible effect of the arrival at that end of the unit, not checked.

**Two more series with everything new: the travel pose with the camera forward, the station at 0.30 m, the look pose within the pan limit, the swing-aware choice of the pre-grasp and the branch-first look (`--best-pregrasp --localize`, tight AMCL), on new draws (seeds 1 and 2; the draws are not those of the series above, so the comparison is not paired).** Seed 1: **29 of 30 held (97 percent), 26 within 10 mm**; seed 2: **27 of 30 held (90 percent), 25 within 10 mm**; together 56 of 60 held and 51 within 10 mm, the same count as the 56 of 60 above, with other failures. By level, seed 1 and seed 2: level 1 9 of 9 and 9 of 9 (against 8 of 9 and 7 of 9 before), level 2 8 of 8 and 6 of 8, level 3 12 of 13 and 12 of 13. The shift was found in 30 and 29 attempts, none ambiguous (27 of the 30 of seed 1 at the first look, 0.30 m; all 29 of seed 2); against the true one the difference along the face has a median of 2.9 and 5.2 mm, a 95th percentile of 10.0 and 9.0, a maximum of 14.2 and 12.6 mm, with a true shift of up to 79 and 119 mm. Arrival: 12 to 82 mm from the station (median 33) and 9 to 152 mm (median 42), a heading off by a median of 2.8 and 3.4 degrees; the belief off along the shelf by a median of 21 and 17 mm and at most 69 and 105 mm (2 and 4 attempts with 50 mm or more, 0 and 1 with 80 mm or more), a median of 52 and 51 mm before the five updates and 21 and 17 after. Tilt of the held: median 7.6 and 2.9 degrees; 14 and 14 below 5, 6 and 3 between 5 and 20, 9 and 10 above 20 (the cereal boxes and the juice cartons): the tilt did not change. Ends of the unit (n1, n5): 13 of 14 and 9 of 11. One series takes about 35 minutes. **The four not held:** (1) seed 1, a water bottle at L3.n5 (grasp_dz 0.04 m): the shift was found (38.0 mm against a true 37.8) and the estimate was 1.2 and 1.8 mm from the true pose, but no pre-grasp had a complete approach (`no solution with a complete approach`; the branch of the nominal slot was not found, the four candidates had 20, 219, 20 and 107 points on the line); a cereal box and a soup can at the same slot were held. (2) Seed 2, a cereal box at L3.n1, the same message (the shift 39.7 mm against a true 38.3, the estimate 4.3 and 0.7 mm off; the robot stopped 54 mm from the station): two pre-grasps in a row at the ends of level 3, the place where the study of the gap (7.4) found the least margin to an arrival error. (3) Seed 2, a juice carton at L2.n1: no estimate; at the three looks (0.30, 0.22 and 0.38 m) the best shift explained one article and not the slot, with one or two of the three or four masks cut by the border of the image, so the vote had too little to go on; it is the one failure of the localization in the 60 attempts. (4) Seed 2, a soup can at L2.n4: the approach, the controller failed with code -4 and the TCP 13.1 mm from the goal, the kind of failure seen with the true pose; the belief was off by 49 mm. Of the 60 attempts of the two series, 5 were held but more than 10 mm from the intended position: three jam jars and a soup can at 14.7 to 15.0 mm, and a soup can at 10.4 mm. What this shows: the changes of the pose and of the station removed the failures at level 1 (18 of 18) and did not make the total better, because two pre-grasps now fail at the ends of level 3; what it does not show: that the number moved, since one series of 30 is within what chance does, and the draws differ from series to series.

**Two series with the station in front of the slot** (the slot drawn from the seed before the robot is driven there, `SLOT=draw`; the rest as above: `--best-pregrasp --localize`, tight AMCL, travel pose forward, 0.30 m; the slots are drawn uniformly among the 15 of the unit, not from the stock, so the draws are not those of the earlier series either). Seed 1: **28 of 30 held (93 percent), 23 within 10 mm**; seed 2: **30 of 30 held, 24 within 10 mm**; together **58 of 60 held (97 percent), 47 within 10 mm**. Level 1: 10 of 12 and 10 of 10, level 2: 7 of 7 and 11 of 11, level 3: 11 of 11 and 9 of 9. The ends of the unit (n1, n5): 16 of 17 and 11 of 11, those of level 3 all held (11 of 11: five and six, against the two pre-grasps that failed there in the two series with the station of the unit). The shift was found in 28 and 30 attempts, none ambiguous (23 and 29 at the first look); against the true one the difference along the face has a median of 3.8 and 2.3 mm, a 95th percentile of 11.7 and 8.8 and a maximum of 15.6 and 12.9 mm, with a true shift of up to 91 and 86 mm. Arrival: 5 to 128 mm from the station (median 40) and 4 to 103 mm (median 45), a heading off by a median of 4.0 and 2.9 degrees; the belief off by a median of 23 and 21 mm and at most 88 and 80 mm (5 and 5 attempts with 50 mm or more, 1 and 0 with 80 mm or more). Tilt of the held: median 1.9 and 2.1 degrees, 21 and 20 below 5, 3 and 6 between 5 and 20, 4 and 4 above 20; these draws had 9 cereal boxes and 7 juice cartons in 60 against 16 and 13 in the series before, so the lower count above 20 degrees is the draw, not a change. Not clean (more than 10 mm from the intended position): eight jam jars at 10.4 to 15.0 mm (the same limit as before), three of the other classes at 10.9 to 12.8 mm. **The two not held** are both water bottles at level 1 (n5 and n4, grasp_dz 0.04 m, seed 1), the robot 84 and 62 mm from its station, with no estimate (`vision_failed`): at the look pose of 0.30 m the planner failed three times in a row (-2) with each of the two poses tried, at 0.22 m the look pose was reached but one article only was detected (two or three masks cut by the border of the image) and the vote did not include the slot, at 0.38 m there was no collision-free IK. They are not pre-grasp failures, and no other class failed that way in these 60. **Cause found afterwards** (`scripts/look_plan_check.py`: the arm planned from the travel pose to every collision-free look pose of the slot, with the robot off its station by the measured arrival, 82 mm along the aisle, 17 mm away from the shelf and 10 degrees of heading at L1.n5): the look pose of the natural branch fails with code -2 in 0.02 s, with 15 s and 10 attempts too, while the goal and the start are valid and the other look pose plans. `move_group` says `Computed path is not valid. Invalid states at index locations: [36] out of 54`, the contact being the left finger and `shelf_S1`: the boards of the shelves are 3 cm thick, the default step of the validity check of OMPL (0.01 of the extent of the joint space, some 8 cm of a fingertip at the end of the arm) lets a path go through a board between two checked states, and the planning pipeline refuses the path afterwards. It depends on where the robot stops, so it is not the station in front of the slot as such. **Corrected** with `longest_valid_segment_fraction: 0.001` in `ompl_planning.yaml` (the plan then takes 0.1 s instead of 0.02 s): the same request plans in 0.12 s, and over the 15 slots of S1 with seven arrival errors each (0 and +-80 mm along the aisle, up to 10 degrees of heading, 557 and 564 look poses with a collision-free IK) the plans that fail with -2 go from 28 to 1 (the one left, S1.L2.n3 with the robot 80 mm off and 10 degrees of heading, is the natural look pose with no path in the scene). The same mechanism may be behind the juice carton at L1.n5 whose planner returned an invalid path for the three candidates in the series with the true pose; not checked. Repeated with the simulation and Nav2 in the next paragraph. What this shows: the failures at the ends of level 3 are gone; what it does not show: that the station in front of the slot is better overall, since there are 60 attempts on one unit and the two failures that remain were a planning step, not the station.

**Verification of the planner fix** (seed 1 again, `SLOT=draw`, the same settings and the same slots drawn from the seed, with `longest_valid_segment_fraction: 0.001`; the arrival of Nav2 is not the same, so this is not a paired comparison): **29 of 30 held (97 percent), 25 within 10 mm** (28 and 23 before). Level 1: 11 of 12 and 10 within 10 mm, level 2: 7 of 7 (4 clean), level 3: 11 of 11 (all clean). The look pose that failed with -2 in 5 attempts of the series before (a cereal box, a juice carton and three water bottles, all at level 1) was reached at the first try in all 29 attempts with an estimate, and only one attempt needed more than one look (7 before). Arrival: 5 to 94 mm from the station (median 37), the belief off by a median of 18 mm and at most 71 mm. Tilt of the held: median 1.4 degrees, 18 below 5, 5 between 5 and 20, 6 above 20 (8 cereal boxes and juice cartons in this draw). Not clean: four jam jars at 12.8 to 15.0 mm. **The one not held** is again a water bottle at level 1 (n4, grasp_dz 0.04 m, the robot 34 mm from its station, 3 degrees off), but with another cause: `no collision-free IK` at the look pose of 0.30 m (and at 0.38 m), one article only in the view at 0.22 m. With `look_plan_check.py` and the neighbours in the scene the look pose of this slot has two IK solutions with the robot on its station and with it 10 mm nearer to the shelf, none from 17 mm nearer (the belief was 17 mm, the true pose 29 mm) and none at a look distance of 0.32 m or more on the station or nearer to the shelf, while with 0.24 to 0.28 m it has two solutions at 0, 17 and 30 mm nearer. So the look pose of level 1 is at the edge of what the arm reaches, and the order of the distances tried (0.30, 0.22, 0.38 m) goes the wrong way for it. **Changed** in `pick_oracle.py`: the distances are now 0.30, 0.26, 0.22 and 0.38 m (0.26 m has an IK solution where 0.30 m has none, and sees more articles than 0.22 m). **Seed 1 once more** (same slots, another arrival): **30 of 30 held, 26 within 10 mm**; level 1: 12 of 12 (11 clean), level 2: 7 of 7 (4 clean), level 3: 11 of 11 (all clean). Every attempt found its estimate at the first look, at 0.30 m, so the new distance was not used: the water bottle at L1.n4 arrived 94 mm along the aisle and 39 mm away from the shelf (the failure was with 17 to 29 mm nearer to it), and it was held. The 0.26 m look is therefore checked only with `look_plan_check.py` (IK at 0, 17 and 30 mm nearer), not yet with the simulation. Arrival: 8 to 94 mm from the station (median 38), the belief off by a median of 28 mm and at most 92 mm. Tilt of the held: median 1.8 degrees, 21 below 5, 4 between 5 and 20, 5 above 20. Not clean: four jam jars at 13.7 to 15.0 mm.

**The inventory read from the images** (`--inventory-images`, `localize_images` in `scripts/localization.py`). Until here the class of every slot of the face came from the stock of the simulator. Now the function is given the slots (their nominal places on the face and level) and only the class of the article to pick, which the task says; the class of the other slots is read from the image. A shift of the believed map is a candidate when it puts detections on slots; the score is the number of detections that fall on a slot, the slot to pick must be taken by a detection of the expected class, and the smallest shift wins. Without the classes a shift cannot be told from the same shift plus one pitch (0.2 m) except at the ends of the view, so the candidates are limited to half a pitch, `HALF_PITCH` = 0.10 m (with the inventory given the limit was the 25 cm of the search). The function returns the inventory it read (slot and class) for the slots in the view. `localize_check.py --images --empty 0.3` on made-up shelves (the same 5 slots, 30 percent of them empty, noise of 5 mm): with shifts up to +-9 cm 4000 of 4000 right, none ambiguous, and the inventory right in 8450 of 8450 slots read; at +-10 cm 3963 right, 4 wrong and 33 without a result (15 flagged ambiguous); at +-12 cm 3394 right, 68 wrong and 538 without a result; at +-20 cm 2049 right, 209 wrong and 1742 without a result (with the inventory given, `localize` is right in 3985 of 4000 at +-20 cm). **Series** (seed 1, `SLOT=draw`, `--best-pregrasp --localize --inventory-images`, tight AMCL, the same slots as the two before, another arrival): **30 of 30 held, 26 within 10 mm**, as with the inventory given. Level 1: 12 of 12 (11 clean), level 2: 7 of 7 (4 clean), level 3: 11 of 11 (all clean). The images gave an estimate in all 30 attempts; 38 slots were read in the 30 attempts (1 to 3 each, the ones in the view) and all 38 had the class of the stock. The shift along the face against the true one (the believed map against the simulator's): the difference has a median of 3.8 mm, a 95th percentile of 11.1 and a maximum of 15.5 mm, with a true shift of up to 64.5 mm; none ambiguous. **Two attempts needed the 0.26 m look**, both at L1.n5 (a cereal box and a jam jar): at 0.30 m there was `no collision-free IK`, at 0.26 m the look pose was reached and both were held (the jam jar at 15.0 mm from the intended position, as the other jam jars). So the new distance, which before had been checked only offline, works in the simulation in these two. Tilt of the held: median 0.7 degrees, 20 below 5, 5 between 5 and 20, 5 above 20 (8 cereal boxes and juice cartons in this draw). Not clean: four jam jars at 12.5 to 15.0 mm. Arrival: the belief was off along the shelf by a median of 13 mm and at most 63 mm, so the series did not test the shifts near half a pitch (the true shift went to 91 mm in the series with the inventory given): the limit of 0.10 m is checked only offline. What this shows: with the shift as small as in this arrival the inventory can be read from the images and the pick holds as before; what it does not show: that it holds with a shift of 9 to 10 cm, nor on a shelf with many articles of the same class in a row (the figures offline use random classes). The neighbours of the planning scene are still placed from the simulator's stock.

**The neighbours of the planning scene from the images** (`--scene-images`, with `--localize`; `scene_from_images` in `pick_oracle.py`). Before the first look the scene has only the article to pick, at the nominal place of its slot (the task says which one it is); after it, every detection that falls on a slot of the board (within 5 cm of it once the shift is applied) becomes a box or a cylinder of the size of its class, at the x, y and yaw of the estimate (already in the believed map) and at the height of the board of the slot. What the image does not show is not in the scene. **Series** (seed 1, `SLOT=draw`, `--best-pregrasp --localize --inventory-images --scene-images`, the same slots as the series before, another arrival): **30 of 30 held, 25 within 10 mm**; level 1: 12 of 12 (11 clean), level 2: 7 of 7 (3 clean), level 3: 11 of 11 (all clean); not clean: four jam jars at 10.6 to 15.0 mm and a soup can at 11.2 mm. No article other than the one picked moved. The scene held the article to pick and 0 to 2 neighbours, against about 5 on the face in the stock (those beyond the view, 0.4 m or more from the slot, were missing); all the classes read were those of the stock. The detections that no slot of the face took were 44 in the 30 attempts; a run of 5 attempts with their positions logged showed them to be articles of the **other face** of the unit, 0.50 to 0.53 m behind (the camera sees through the unit), and the matching now takes every slot of the board, both faces. Eight attempts with that change: 8 of 8 held and clean, in 7 of them every detection found a slot, in the other one a parcel was 422 mm from the nearest slot. **Control without any article in the scene** (`--no-scene-neighbours`, same seed and settings, `--inventory-images`): **30 of 30 held, 26 within 10 mm**, level 1: 12 of 12 (11 clean), level 2: 7 of 7 (4 clean), level 3: 11 of 11 (all clean); no article moved either, and the number of attempts with a pre-grasp that needed more than one plan is the same (2 of 30 in each of the three series: stock scene, scene from the images, no articles). What this shows: the neighbours of the scene can be taken from the images and the pick holds as before; what it also shows is that on these draws the articles in the scene do not matter, since the series without them is the same (the articles are 0.2 m apart, about 13 cm free, and the fingers open to 8.5 cm; the arm that hit its neighbours, 1 held of 25, was before the choice of the pre-grasp by the length of the line and the other changes). What it does not show: whether they matter with a neighbour closer than the pitch of the shelves, which was not tried; the arrival of Nav2 differs from series to series, so the three are not paired; the yaw of the boxes in the scene is the estimate of the images (median error 4.3 degrees).

The slide of the base (wheels do not hold it, odometry does not see it) did not show in these figures: between the start and the pre-grasp the base moved 9 to 14 mm in the two attempts examined (the empty ones); in the run of 8 with the tight file one attempt (a cereal box) moved 0.27 m and 54 degrees during the pre-grasp, with the believed pose unchanged. The sliding is an artefact of the simulation (a real base holds with its brakes), not corrected.

### Known limits (step 8, so far)

- Walls, boards and floor are plain colours, so stereo has no depth there (a real D435 has an infrared projector for this reason): the depth is to be used inside the mask of the article, where there are labels and stripes.
- Outside the textured regions the matches are wrong: the depth must be filtered with the mask of the recognition, not used as it comes.
- The noise of the images and the bias of the stereo depth are assumptions, not calibrated.
- The labels mark some articles that the image does not show, hidden by a light surface: 0.15 percent of the article pixels in the test split (a count of the pixels on near-white surfaces, so a lower bound) and 2.4 percent in the robot images, behind the shape. The cause was not investigated.
- The test with the robot has 36 images, a small sample; the figure is an indication, not a guarantee.
- The pose on the robot's camera is measured with the pose of the robot taken from the simulator and the arm's transforms from the joints, which in the simulation are exact: the error of a calibration of the camera on the arm, and of the joint readings of a real arm, is not in the figures. 64 views, 59 estimates.
- The grasp was measured on one unit (S1). With the robot put back at its station by teleport: 29 of 30 with the true pose, 30 of 30 from the images. With the robot arriving by Nav2 and the shift of the believed map found from the images: 29 of 30 and 27 of 30 on a second set of draws (20 of 30 when the article was looked for near where the believed pose projects the slot). With the travel pose forward, the station at 0.30 m and the branch-first look pose, on two more sets of draws: 29 of 30 and 27 of 30 again; with the station in front of the slot to pick, 28 of 30 and 30 of 30. It needs the inventory (the class of every slot of the face), read from the simulator's stock or, with `--inventory-images`, from the images (30 of 30 held on one more series, with a shift of the believed map of up to 65 mm and a limit of 100 mm), and a few articles of different classes in the image to tell the slots apart: with 4 or 5 of the same class in a row the shift is ambiguous (5 in 4000 made-up cases) and the attempt gives up. It corrects the direction along the face; across it the LiDAR error (up to 28 mm in the series) is not corrected. The base can slide in the simulation. The neighbours are static in the simulation, and their place in the planning scene is taken from the simulator (moved by the shift found) or, with `--scene-images`, from the images (30 of 30 held, the same as without any article in the scene).
- An article whose mask has stereo depth on less than 60 percent of its pixels gives an unreliable pose (the errors in depth reach tens of millimetres): the task has to ask for another view.
- Training, validation and test come from the same warehouse: they differ by arrangement and pose, not by shelves, walls or lighting. The test measures the recognition of unseen fillings, not of another environment.
- The stereo node is on demand: it gives one depth image per request, not a stream.
- The first reading of the real time factor after the start is lower; the figures above are from runs of a few minutes, not averages over a long time.

## Step 9: task logic (in progress)

The mission: take the article of a shelf slot, carry it to the packing table and drop it in a bin. The tree decides the order, the retries and the other bin; the skills do the work.

### The drop in a bin (9.1)

`scripts/place_check.py` (MoveIt alone, static scene): with the front grasp of the shelves (tool horizontal, pointing at the table) and the article held with its centre at the TCP, the lowest point of the article has to clear the rim of the bin (0.87 m) by 3 cm, so the TCP is at 0.95 to 1.03 m (0.98 for a 16 cm article, 1.03 for the water bottle, grasped 4 cm above its centre). For each gap between the platform and the table, each position of the robot along the table and each drop point over the bin: all the collision-free IK solutions of the pre-place pose (0.10 m before the drop point, on the line), the ones whose straight line to the drop point is complete, and whether OMPL plans to them from the travel pose. With the station of the table at the old 0.25 m gap only the near edge of the bin was reached; with **0.15 m** (`PLACE_GAP`) the drop point 4 cm from the centre of the bin towards the robot has 2 to 6 solutions for every article, also for the water bottle, with the robot up to 0.10 m off along the table; the depth margin is about 5 cm (the localization along the normal of a flat face is the one the LiDAR fixes, under 3 cm). The station is in front of the bin, not of the table. The tool pitched down gave no more than the horizontal one.

`Picker.carry` (the article attached to the TCP in the planning scene, the arm at the travel pose, the simulator says whether it is held), `Picker.place` (the pre-place pose from IK seeds with wrist 1 turned the long way, the line over the rim at a speed scale of 0.25, the gripper opens, the arm goes back) and `pick_oracle.py --deliver bin_1,bin_2` (one drawn) are the code. Series on the 12 slots of seed 1 (`SLOT=draw`, the settings of the best pick series, a fresh world each, the bin drawn): **7 of 12 delivered** at first; the three cases without a pre-place pose had only the IK branch with an incomplete line, and with seeds of the good posture they were delivered (**10 of 12**); the same with the article attached to the scene (**10 of 12**); and, after a mistake in how the held article was checked, **11 of 12**: the one lost was a water bottle at L1.n5 that fell while the arm went to the travel pose (1.29 m from the TCP). The mistake: the article was compared with "below the TCP by `dz`" in the world, but with the tool turned for the travel pose the tool x axis is not up and the article is 2 `dz` from there (80 mm for the bottle, 30 for the jam jar): it was not slipping; with the position taken along the tool x axis the bottle is 14.5 mm from where it should be, turned by 147 degrees in the pads. In the last series the articles ended on the floor of the bin, 1 to 7 cm from its centre in x and at most 6 cm in y. The bin is judged from the pose of the article in the simulator.

### The tree and the skills (9.2)

`amr_manipulator_task` has five actions, one per skill (`NavigateTo`, `PickArticle`, `CarryArticle`, `PlaceInBin`, `Recover`), a C++ executor (`mission_executor`, BehaviorTree.CPP 3.8.7 from the ROS 2 packages: every leaf is a `StatefulActionNode` that sends the goal to the action, is RUNNING until the result arrives and writes the outcome to a port) and the tree `trees/fetch_and_deliver.xml`:

- the drive to the shelf is tried twice;
- the pick (`PickWithRecovery` subtree) is tried three times: again as it is, then after opening the gripper and stowing the arm, then after driving to the station again;
- the carry checks that the article is held;
- the bin (`DeliverWithFallback` subtree): the first; if the drive or the placing fails the arm goes back to the travel pose with the article and the other bin is tried;
- whatever happens the arm ends in a safe state (`Recover`).

`scripts/skill_server.py` (Python) has the five actions as servers and calls `Picker.pick/carry/place` of `pick_oracle.py` with the setting of the best series (station in front of the slot, best pre-grasp, the shift of the believed map, the inventory and the neighbours from the images). One skill at a time; each writes what it measured to `/tmp/skills_trace.jsonl`. The simulator is used for what the robot cannot do: to find the model of the stock in a slot (to make it a dynamic body when it is picked) and to judge. `scripts/mission.sh` starts the simulation, Nav2 with the tight AMCL, MoveIt and the stereo node, stocks the shelves and picks the ordered article (`mission_setup.py`), starts the server and runs the executor; `scripts/mission_series.sh` repeats it with a fresh world and the bin alternated.

**The tree without the simulation** (`scripts/skill_stub.py` answers to every skill at once with failures that can be injected, `scripts/bt_check.sh`): everything works: the six skills in the order of the tree; the first two picks fail: the third (after driving again) is delivered; all the picks fail: three tries and the safe end, the mission fails; bin_1 cannot be used: the arm goes back to the travel pose and the delivery is in bin_2; neither bin: the mission fails and the gripper opens; the drive to bin_1 fails once: bin_2; the article is lost in the carry: the mission fails at once, no retry. **A mistake found there:** in BehaviorTree.CPP 3 a `SubTree` maps the keys of its scope to the keys of the parent with the name of the parent's key without braces (`target_slot="slot"`); with braces (the syntax of version 4) the port of every leaf in the subtree was empty and each pick and drive failed with "a port of the goal is missing"; the first two missions with the simulation failed for that.

**With the simulation** (unit S1, the setting above): the first mission with the tree that worked gave a juice carton that fell during the drive (`lost_in_drive`: the article was farther than 6 cm from the TCP at the table), the tree tried the other bin, saw that the article was lost, opened the gripper and ended in the safe state; the same mission again was delivered (the juice carton tilted by 25 degrees in the pads, 2.5 mm from the TCP at the table, in the bin at 62 mm from its centre in x and 0 in y). **Series of 10 missions** (seed 1, the slot drawn from the seed, bin_1 and bin_2 alternated, a fresh world each): **10 of 10 delivered**, one pick and one place in each (the retries were not needed), 173 to 197 s a mission (about 8 minutes with the start of the simulation); 3 cereal boxes, 2 parcels, 2 water bottles, 2 jam jars, 1 juice carton. **Failures injected in the simulation:** a bin that does not exist (`bin_9`, `bin_2` as the other): the drive fails (the server answers with the exception), the arm goes back to the travel pose, bin_2: delivered, mission succeeded in 180 s; a class that is not the one in the slot (a cereal box asked at the slot of a water bottle): the look finds no article of that class on the slot at 0.30, 0.26 and 0.22 m (and no IK at 0.38 m), three tries of the pick (the third after driving again), each `vision_failed`, the gripper opens and the mission fails in 163 s. (A first try with a class that was not the true one did not fail: the simulator's adapter made the article again with the class asked for; it now makes it with the true class and the vision looks for the one asked for.) **An article lost for real:** a water bottle at L1.n5, held within 2.7 mm and 1.2 degrees, was found 3.6 m from the TCP, lying, with the gripper empty, when the robot arrived at the bin; the tree tried the other bin, saw the loss and ended in the safe state, the mission failed. The same slot gave a bottle that fell in the stow in the series of the drop (1.29 m) and two that were delivered; it was delivered in the series of 10. The cause is not known. **Looked for afterwards** (`pick_oracle.py --carry-laps N`: with the article held the robot drives between the station of its slot and the one of the bin, and after every drive the distance of the article from where it is held and the state of the gripper are noted): a water bottle at L1.n5, 2 worlds, 4 and 15 laps: **38 drives (about 215 m) without losing it**, the gripper in GRASP at 60.0 mm and 25.1 and 24.9 N all along, the distance from the grasp growing by 0.2 mm a drive (20.1 to 21.7 mm in 8 drives, 6.3 to 11.8 mm in 30); and **8 missions in a row with a water bottle at L1.n5** (`scripts/bottle_missions.sh`, the skill server noting the pose of the robot and of the article every second of each drive): 8 of 8 delivered, no recovery of Nav2 in any of the 5.3 to 5.8 m drives. So Nav2's accelerations (2.5 m/s squared, 0.4 m/s) do not drop the bottle by themselves, and the two real losses (1.29 m from the TCP in the stow in the series of the drop, 3.6 m in a mission) were not seen again in 10 further bottle passes: of the passes with a bottle at that slot, 2 in about 17 lost it, a rate with a wide interval (from 1.5 to 36 percent) and no known cause. A gentler Nav2 (`config/nav2_params_gentle.yaml`: 0.25 m/s, 0.5 m/s squared) was written and not needed to be tried; the pose of the robot and of the article every second is in `/tmp/skills_trace.jsonl` for the next time it happens.

**What this shows and what not.** The tree runs the whole sequence, with the retries and the other bin as designed, on the simulation, and 10 of 10 missions were delivered on one draw; one article lost in the drive and one in the stow were seen on bottles of one slot, so the figure is not a rate (10 missions, one unit, one seed; the arrival of Nav2 is not the same each time). The drop is judged by the simulator; the pick is the one of step 8 with its limits (the cereal boxes and the juice cartons held and tilted, the jam jars 12 to 15 mm off); no recovery for a lost article; one or two articles at a time (see 9.3).

### Two articles in a mission (9.3)

`trees/fetch_two.xml` runs the subtree `FetchOne` (drive to the shelf, pick with the retries, carry, bin with the other bin as fallback, stow) twice, once per order, each inside a `ForceSuccess` so the second order is tried also when the first failed, and ends with the condition `AllDelivered` (C++, in `mission_executor.cpp`): the mission succeeds only if both orders were delivered. The orders are parameters (`o1_slot`, `o1_class`, `o1_bin`, `o1_alt`, and the same with `o2_`); `mission_setup.py --orders 2` draws two different slots (and classes) and `ORDERS=2 scripts/mission.sh` / `mission_series.sh` run them, the first to `bin_1` and the second to `bin_2`. The four two-order cases of `scripts/bt_check.sh` (with the stub, no simulation) behave as designed.

**A finding in the planning scene.** In the first series of six missions, three failed because the second article had no place pose (`no_place_pose`, no complete line over the rim from any of the IK solutions). Two causes: the series script sent both orders to `bin_2` on the odd missions (my mistake), and, behind the failures, the planning scene: removing an attached object with `REMOVE` only detaches it, and it stays in the world as a box where the TCP was (checked with `scripts/attach_check.py`: after the REMOVE the object was still a world object); after the first drop, a box lay over the bin and blocked the line of the second. `attach_article(attach=False)` now also removes the world object `held_<name>` in the same request; with the check the object is gone after the step.

**Series of 6 missions, two articles each** (unit S1, seed 2, the settings above, a fresh world each, the first article to `bin_1`, the second to `bin_2`): **5 of 6 delivered, both articles**, 441 to 645 s a mission (the time of the executor, without the start of the simulation). The first series (before the two corrections) gave 2 of 6. In the failed mission (a soup can at L1.n3 and a juice carton at L2.n1): the soup can was held at 2.9 mm and tilted by 14.2 degrees at the pick, and after the arm went to the travel pose it was 73 mm from the TCP (53 mm down along the tool), the gripper still in `GRASP`; at both bins the arrival check (`HELD_ARRIVED`, 6 cm) answered `lost_in_drive`, so the soup can was not placed, and the juice carton was delivered to `bin_2`; the mission failed because of the first order. It is the same draw as in the first series (the seed gives the same slots), so it is one case seen twice, not two. It is not a loss: the can is held but has slid in the pads, and the 6 cm threshold, chosen for the article that falls, is what stops it; I did not try to place it with the larger displacement. In the first mission 4 picks were needed (the jam jar at L2.n4 was reported `lost` twice, 220 mm from the TCP with the gripper in `GRASP`, then held at 10.5 mm on the third try; the second order took one); one parcel arrived at the bin 28 mm down along the tool and was delivered. What is not tested: the two articles in the **same** bin (the two orders went to different bins), so the world object removed after the drop was checked with `attach_check.py` but not by a mission that drops twice in one bin; and the standing article that sticks above the rim.

## Roadmap

- **Step 7 (rest):** the gripper in the sequence approach, grasp, retreat.
- **Step 8 (rest):** the tilt of the cereal boxes and of the juice cartons (they are held but tilted by 28 to 38 degrees when the box is turned by 7 degrees or more, which the straight approach ignores; approached turned by the true yaw 8 of 8 boxes were upright; the yaw from the images, with a 95th percentile error of 20 degrees, is not accurate enough yet, and the grasp does not tolerate an error of 3 degrees: 5 of 12 upright with errors of 3 and 6 degrees; a weaker grip and a slower retreat do not change it; the box is held and tilted, and the grasp is left as it is), the vision that finds no estimate when the border of the image cuts the neighbours and the look pose is not reached (two water bottles at level 1 in the series with the station in front of the slot; one juice carton at L2.n1 before), and the pose of the boxes in the scene (the neighbours of the planning scene can now be taken from the images, `--scene-images`, with no change in the result). The grasp holds 29 of 30 with the true pose, 30 of 30 from the images with the robot put back by teleport and 56 of 60 from the images with Nav2 (two series), again 56 of 60 with the last settings (two more series) and 58 of 60 with the station in front of the slot (two more), 29 of 30 with the planner fix and 30 of 30 with the new look distances (one more series each, seed 1), 30 of 30 with the inventory read from the images (one more, seed 1), at one unit (S1).
- **Step 9 (rest):** the article lost (two bottles at L1.n5 in about 17 passes, one in the stow and one in the drive, not seen again in 46 drives and 10 passes; no recovery in the tree: the article lies somewhere the robot does not know), the article held but slid in the pads (a soup can tilted at the pick: 73 mm from the TCP after the travel pose, refused by the 6 cm check), two articles in the same bin, the other units, a place where the drop does not need the simulator to judge it (the bin is judged from the pose of the article in the simulator, the robot has no sensor for it).
