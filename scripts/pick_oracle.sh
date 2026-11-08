#!/usr/bin/env bash
# Pick articles with the true pose (no vision, no Nav2): the robot is put at the station of a shelf unit, with an exact map.
#   usage: [ALONE=1] [LOG=arm.csv] pick_oracle.sh [UNIT=S1] [TRIALS=6] [OUT=/tmp/pick_oracle.json] [SEED=1]
#   NAV=1 (with VISION=1): the robot starts at the origin and Nav2 (AMCL, planner) drives it to the station, as in the real task: nothing is teleported and the transforms are AMCL's; the simulator only judges.
#   GUI=1: the window of Gazebo is shown (needs the display of the container); with EXTRA='--hold 8' the article stays in the gripper 8 s after the retreat.
#   NAVPARAMS=precise (with NAV=1): the AMCL parameters of config/nav2_params_precise.yaml.
#   SLOT=S1.L3.n5: pick the article of that slot from the station in front of it (the robot is driven or spawned there); SLOT=draw: the slot is drawn from SEED before the start (the station of the unit is the default).
#   VISION=1: the pose of the article from the images (network, stereo) instead of the true pose; ALONE=1: one article on the shelves, no neighbours; EXTRA='--force 40 --grasp-dd 0.01 ...': more arguments of pick_oracle.py; LOG: record the arm controller state (arm_logger.py)
# Run it in the container with nothing else running: it stops at once if a simulation is already up. It starts the
# simulation with the robot spawned at the station, move_group, runs pick_oracle.py (which publishes map to odom from the
# position of the robot in the simulator) and stops only what it started (its own process groups).
UNIT=${1:-S1}
TRIALS=${2:-6}
OUT=${3:-/tmp/pick_oracle.json}
SEED=${4:-1}
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
HERE=$(cd "$(dirname "$0")" && pwd)

BUSY=$(ps -eo pid,args | grep -E "ign gazebo|move_group|parameter_bridge|robot_state_publisher|component_container|stereo_depth_node|ros2 launch|follow_joint_trajectory" | grep -v grep | grep -v "bash -l")
if [ -n "$BUSY" ]; then echo "something is already running, nothing started:"; echo "$BUSY" | cut -c1-100; exit 3; fi

PLACE=$UNIT
if [ "${SLOT:-}" = draw ]; then
  SLOT=$(cd "$HERE" && python3 -c "
import os, semantic_goals as sg
print(sg.draw_slot(sg.load(os.path.join(sg.GAZEBO, 'config', 'semantic_map.yaml')), '$UNIT', $SEED))" 2>/dev/null)
fi
[ -n "${SLOT:-}" ] && PLACE=$SLOT
read -r X Y YAW <<< "$(cd "$HERE" && python3 -c "
import os, semantic_goals as sg
sem = sg.load(os.path.join(sg.GAZEBO, 'config', 'semantic_map.yaml'))
_, (x, y, yaw) = sg.resolve(sem, None, '$PLACE')
print(x, y, yaw)")"
echo "== station of $PLACE: x $X, y $Y, yaw $YAW"

STARTED=""
OWN=""                                           # the simulator can leave its process group: its pids are kept
start() {                                        # start NAME COMMAND...: a process group of its own
  local name=$1; shift
  setsid "$@" > "/tmp/pick_oracle_$name.log" 2>&1 &
  STARTED="$STARTED $!"
}
stop_all() {
  for pid in $STARTED; do kill -TERM -- "-$pid" 2>/dev/null; done
  sleep 5
  for pid in $STARTED; do kill -KILL -- "-$pid" 2>/dev/null; done
  [ -n "$OWN" ] && kill -KILL $OWN 2>/dev/null
  STARTED=""; OWN=""
}
trap stop_all EXIT

wait_for() {                                     # wait_for DESCRIPTION SECONDS COMMAND...
  local what=$1 limit=$2; shift 2
  for _ in $(seq $((limit / 2))); do
    sleep 2
    "$@" > /dev/null 2>&1 && return 0
  done
  echo "timeout waiting for $what (logs in /tmp/pick_oracle_*.log)"; exit 4
}
world_up() { ign topic -e -t /world/amr_world/stats -n 1 2>/dev/null | grep -q real_time_factor; }
has_action() { ros2 action list 2>/dev/null | grep -q "$1"; }

echo "== simulation"
GUIARG=false; [ -n "${GUI:-}" ] && GUIARG=true
SPAWN="x:=$X y:=$Y yaw:=$YAW"                    # NAV=1: the origin, where AMCL expects the robot
[ -n "${NAV:-}" ] && SPAWN=""
start sim ros2 launch amr_manipulator_gazebo sim.launch.py world:=warehouse.sdf gui:=$GUIARG $SPAWN ${VISION:+stereo:=true dataset:=true}
wait_for "the world" 120 world_up
OWN=$(ps -eo pid,args | grep -E "ign gazebo|ruby.*ign" | grep -v grep | awk '{print $1}' | tr '\n' ' ')
wait_for "follow_joint_trajectory" 90 has_action follow_joint_trajectory
sleep 30                                         # the arm goes to the travel pose after the start
if [ -n "${NAV:-}" ]; then                      # NAV=1: AMCL and Nav2, the robot starts at the origin (the initial pose of AMCL)
  echo "== Nav2"
  NAVARGS=""
  if [ "${NAVPARAMS:-}" = precise ] || [ "${NAVPARAMS:-}" = gentle ]; then          # NAVPARAMS=precise: AMCL tighter (config/nav2_params_precise.yaml); gentle: and slower, for an article in the gripper
    NAVARGS="params:=$(ros2 pkg prefix amr_manipulator_gazebo)/share/amr_manipulator_gazebo/config/nav2_params_${NAVPARAMS}.yaml"
  fi
  start nav ros2 launch amr_manipulator_gazebo nav.launch.py $NAVARGS
  wait_for "navigate_to_pose" 90 has_action navigate_to_pose
  sleep 20
fi
echo "== MoveIt"
start moveit ros2 launch amr_manipulator_moveit_config move_group.launch.py
wait_for "move_action" 90 has_action move_action
sleep 15
if [ -n "${VISION:-}" ]; then                   # VISION=1: the stereo node, for the pose from the images
  start stereo ros2 run amr_manipulator_perception stereo_depth_node.py
  wait_for "stereo_depth/compute" 60 bash -c "ros2 service list 2>/dev/null | grep -q stereo_depth/compute"
  sleep 3
fi
if [ -n "${NAV:-}" ]; then
  echo "== Nav2 drives to the station of $PLACE"
  python3 "$HERE/nav_semantic.py" "$PLACE" --timeout 200 || { echo "== the station was not reached"; exit 5; }
  sleep 5
fi
echo "== picking: unit $UNIT, $TRIALS attempts"
if [ -n "${LOG:-}" ]; then                      # LOG=file.csv: record the state of the arm controller (50 Hz) meanwhile
  start logger python3 "$HERE/arm_logger.py" "$LOG"
fi
python3 "$HERE/pick_oracle.py" --unit "$UNIT" ${SLOT:+--slot $SLOT} --trials "$TRIALS" --out "$OUT" --seed "$SEED" ${ALONE:+--alone} ${VISION:+--vision} ${NAV:+--nav} ${EXTRA:-}
code=$?
echo "== pick_oracle.py ended with code $code"
exit "$code"
