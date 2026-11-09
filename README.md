# Semantic Mobile Manipulation

A mobile manipulator built in **ROS 2 Humble** and **Gazebo Fortress**. In a simulated order-picking warehouse a robot (differential-drive base, UR5e arm, parallel gripper, LiDAR, wrist stereo camera) receives an order, drives to the shelf, recognises the article on it, picks it, carries it to the packing table and drops it in a bin. A behavior tree runs the mission; everything below it (control, navigation, planning, perception) is written in this repository or configured for it.

## Demo

Everything runs in one shell, inside the development container (see [Build](#build)):

```bash
cd ~/ros2_ws/src/scripts
GUI=true ORDERS=2 ./mission.sh S1 2001
```

The script starts the simulation with the Gazebo window, Nav2, MoveIt and the stereo node, stocks the shelves of unit `S1`, draws two orders, and runs the mission. It takes about 5 minutes to start and about 8 for the mission, and it stops everything it started when it ends. What happens:

1. The robot drives to the front of the slot of the first article and looks at the shelf with the stereo camera.
2. A segmentation network finds the articles; with the mask and the stereo depth the pose of the article is computed, and the position of the robot along the shelf is corrected from what it sees.
3. The arm picks the article with a horizontal grasp, folds into the travel pose, and the robot drives to the packing table.
4. The arm drops the article over the rim of `bin_1`; then the same for the second article and `bin_2`.
5. The executor prints `MISSION SUCCEEDED` or `MISSION FAILED`.

Options: drop `ORDERS=2` for a single article (`SLOT=S1.L2.n1` chooses the slot); the last argument is the seed that draws the stock and the orders; `GUI` left out runs headless. The weights of the trained network are in the repository (`amr_manipulator_perception/segmentation/runs/base.pt`); `train.py` makes new ones, see [Perception](#perception). The seed `2001` is a draw that was delivered in the series of missions (a jam jar and a parcel, 469 s); the arrival of Nav2 is not the same twice, so it is not a guarantee.

## Results

All figures are from the simulation, on shelf unit `S1`, and are the ones measured; the sample sizes are small and say so.

| What | Result |
|---|---|
| Arm tracking with a 3 kg payload, adaptive controller | RMS error 0.0003-0.0005 rad on lift, elbow, wrist 1 (0.007-0.025 rad without adaptation) |
| Navigation, 36 goals (shelves, benches, zones) | all reached, mean position error 0.091 m |
| Front grasp reach | all 105 pickable slots, station 0.30 m from the shelf |
| Segmentation network (from scratch) | mean IoU 0.994 on the test split, 0.960 on 36 images from the robot's camera |
| Pose of an article from the images | median error along the shelf 2.0 mm (95th percentile 6-7 mm) |
| Pick with Nav2, pose and stock read from the images | 58 of 60 held in two series; 30 of 30 in the latest |
| Delivery into a bin | 11 of 12 |
| Mission, one article (behavior tree, whole chain) | 10 of 10 delivered, 173-197 s each |
| Mission, two articles in two bins | 5 of 6 delivered, 441-645 s each |
| Injected failures | a bin that does not exist: the other bin is used, delivered; a wrong class: three tries, then the arm is stowed and the mission fails |

What the numbers do not say: one shelf unit, one stock, a handful of seeds; the arrival of Nav2 is not the same twice, so runs are not paired; the simulator judges the delivery (the robot has no sensor for "it is in the bin"), and helps with two sim-only things (it makes the article a dynamic body when it is picked, and it tells the skill which article is really in the slot).

## The project, layer by layer

### Robot and simulation
One xacro describes the robot; everything that exists only in simulation sits behind a `use_sim` flag. The base is Gazebo's `DiffDrive`; the arm and the gripper are driven in **effort** mode through `gz_ros2_control`, and all topics go through one bridge file. Footprint (an 8-vertex polygon) and a *travel pose* (arm folded upright over the chassis, camera looking forward, nothing outside the platform) were computed from the kinematics and checked in the simulator.

### Control (`amr_manipulator_control`)
No stock controller: the arm uses **computed torque** with the KDL dynamic model, a quintic reference, friction feedforward and an **adaptive payload estimate** (Slotine-Li, mass at the end effector; it settles at 3.0 kg for a 3 kg load). The gripper has its own controller, a state machine (`OPEN`, `CLOSING`, `GRASP` at 25 N, `EMPTY`) with contact detection by stall and a virtual spring that centres the object. A `FollowJointTrajectory` action wraps the arm controller for MoveIt.

### Sensors
A 2D LiDAR (360 rays, 10 Hz), an RGB-D camera in the wrist and a **stereo pair** (50 mm baseline) with a segmentation camera for the labels. Stereo depth is computed on demand (SGBM): at 10 Hz it would slow the simulation to a third of real time.

### World and semantic map (`amr_manipulator_gazebo`)
A 12 x 8 m warehouse (receiving, storage and packing zones, four doors, seven shelf units, benches, two bins) generated from one layout file. The same file produces the **semantic map**: zones, doors, shelves, bins and every slot with its pose. There are six article classes with no markers: the robot has to recognise them. The stock is random (210 slots, each filled with probability 0.30) and can be redrawn.

### Navigation
slam_toolbox built the map; Nav2 (AMCL, DWB with the footprint polygon) drives to **places by name** taken from the semantic map (`S1`, `S1.L3.n5`, `bin_1`, doors, zones). The pick station is in front of the slot, 0.30 m from the shelf face, a distance chosen from the reach analysis of the arm. In front of a flat shelf face the LiDAR fixes distance and heading, not the position along it, so that is corrected by vision (below).

### Motion planning (`amr_manipulator_moveit_config`)
MoveIt 2 with KDL kinematics and OMPL; the planning scene is built from the SDF world and, for a pick, from the articles seen by the camera (neighbours and the article in the gripper are obstacles). Two things are not left to MoveIt's defaults: pose goals pick far IK branches, so the arm goes to an IK solution seeded near the right posture through a *joint goal*, and straight approach and retreat use Cartesian paths. A reach analysis over all slots chose the grasp: from above is impossible on the shelves, a **horizontal front grasp** reaches every slot.

### Perception
Written from scratch, no pretrained models. A **U-Net** (1.94 M parameters, PyTorch) segments the six classes; it was trained on 3990 synthetic images (320 x 240) made by filling the shelves at random and photographing them from poses like those of the arm. The pose of an article does not come from the network: the mask, the stereo depth and the known shape (a circle or a rectangle seen from above) give it by robust least squares. The shifts of the believed map along the shelf are found by matching the articles seen to the slots of the map, and the stock of a shelf face is read from the image as well.

### Pick and place
`scripts/pick_oracle.py` holds the pick (look, estimate, pre-grasp, line to the article, close, retreat), the carry (the article attached to the tool in the planning scene, a slow drive, a check at every stop that it is still held) and the placing over the rim of a bin (pre-place pose with the wrist turned the long way, a slow straight line, open, retreat).

### Task logic (`amr_manipulator_task`)
A C++ executor (BehaviorTree.CPP 3) runs a tree whose leaves call five ROS actions served by `scripts/skill_server.py`: `navigate_to`, `pick_article`, `carry_article`, `place_in_bin`, `recover`. The tree retries (the drive twice; the pick three times: as is, after stowing, after driving again), falls back to the other bin, always ends with the arm in a safe state, and for two orders runs the same subtree twice and succeeds only if both articles were delivered (`trees/fetch_two.xml`). The tree was first tested against a stub of the skills, with injected failures, before the simulation.

## Build

The development environment is the `Dockerfile` of this repository (ROS 2 Humble, Gazebo Fortress, Nav2, MoveIt 2, BehaviorTree.CPP, PyTorch on top).

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Other launches: `ros2 launch amr_manipulator_gazebo sim.launch.py world:=warehouse.sdf` (simulation), `slam.launch.py` and `nav.launch.py` (mapping and navigation), `ros2 launch amr_manipulator_moveit_config move_group.launch.py` (MoveIt). Stop a simulation with Ctrl+C and check that no `ign gazebo` process is left. Nav2 and slam_toolbox must not run together.

## Repository layout

```
amr_manipulator_description/   URDF/xacro (base, UR5e, gripper), controller YAML
amr_manipulator_gazebo/        launch files, bridge, warehouse world and layout, models, semantic map, saved map
amr_manipulator_control/       ros2_control controllers (arm, gripper), FollowJointTrajectory server
amr_manipulator_moveit_config/ MoveIt 2 configuration and planning scene
amr_manipulator_perception/    stereo depth, segmentation network (train, evaluate), pose from mask and depth
amr_manipulator_task/          skill actions, behavior tree and executor
scripts/                       the mission and the tools it uses (pick, navigation by name, stock, world generation)
```

## Known limits

- One shelf unit and one stock were measured; no real hardware.
- The delivery is judged by the simulator. The robot does not sense that the article is in the bin.
- An article that falls is not recovered: a water bottle was lost twice in about 17 passes at one slot (cause not found, not seen again in 46 further drives), and the tree ends the mission in a safe state.
- A tilted article can slide in the gripper during the drive: a soup can ended 73 mm from where it was held and the 6 cm check refused to place it (the one failed mission of the two-article series).
- Two articles were delivered to different bins; two in the same bin was not tested.
- Tall boxes (cereal, juice carton) are held but tilted when turned by 7 degrees or more.

## Future work

- Recover a lost or slid article (re-grasp, or drop it and try again).
- Sense the delivery (camera over the bin) instead of reading the simulator.
- More shelf units and longer orders.
- A real robot.
