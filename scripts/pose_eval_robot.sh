#!/usr/bin/env bash
# Measure the pose of the articles from the camera of the robot (arm on the wrist, Nav2 to the stations).
#   usage: pose_eval_robot.sh [OUT=/tmp/pose_robot.json] [TARGETS=4] [RESTOCKS=4] [UNITS...]
# Views = units x TARGETS x RESTOCKS (4 units, 4, 4: 64 views). Run it in the container, with nothing else running: it
# stops at once if a simulation is already up. It starts the simulation, Nav2, MoveIt and the stereo node, runs
# pose_eval_robot.py, prints the report and stops only what it started (its own process groups).
OUT=${1:-/tmp/pose_robot.json}
TARGETS=${2:-4}
RESTOCKS=${3:-4}
shift $(( $# < 3 ? $# : 3 ))
UNITS=${*:-S1 N2 S3 N1}
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
HERE=$(cd "$(dirname "$0")" && pwd)

BUSY=$(ps -eo pid,args | grep -E "ign gazebo|move_group|parameter_bridge|robot_state_publisher|component_container|stereo_depth_node|ros2 launch|follow_joint_trajectory" | grep -v grep | grep -v "bash -l")
if [ -n "$BUSY" ]; then echo "something is already running, nothing started:"; echo "$BUSY" | cut -c1-100; exit 3; fi

GROUPS_STARTED=""
launch() {                                       # launch NAME ARGS...: a process group of its own, so it can be stopped alone
  local name=$1; shift
  setsid ros2 launch "$@" > "/tmp/pose_eval_robot_$name.log" 2>&1 &
  GROUPS_STARTED="$GROUPS_STARTED $!"
}
stop_all() {
  for pid in $GROUPS_STARTED; do kill -TERM -- "-$pid" 2>/dev/null; done
  sleep 5
  for pid in $GROUPS_STARTED; do kill -KILL -- "-$pid" 2>/dev/null; done
  GROUPS_STARTED=""
}
trap stop_all EXIT

wait_for() {                                     # wait_for DESCRIPTION SECONDS COMMAND...
  local what=$1 limit=$2; shift 2
  for _ in $(seq $((limit / 2))); do
    sleep 2
    "$@" > /dev/null 2>&1 && return 0
  done
  echo "timeout waiting for $what (logs in /tmp/pose_eval_robot_*.log)"; exit 4
}
has_action() { ros2 action list 2>/dev/null | grep -q "$1"; }
world_up() { ign topic -e -t /world/amr_world/stats -n 1 2>/dev/null | grep -q real_time_factor; }

echo "== simulation"
launch sim amr_manipulator_gazebo sim.launch.py world:=warehouse.sdf stereo:=true dataset:=true gui:=false
wait_for "the world" 120 world_up
sleep 15
echo "== Nav2"
launch nav amr_manipulator_gazebo nav.launch.py
wait_for "navigate_to_pose" 90 has_action navigate_to_pose
echo "== MoveIt"
launch moveit amr_manipulator_moveit_config move_group.launch.py
wait_for "move_action" 90 has_action move_action
sleep 20                                         # AMCL and the planning scene settle
echo "== stereo node"
setsid ros2 run amr_manipulator_perception stereo_depth_node.py > /tmp/pose_eval_robot_stereo.log 2>&1 &
GROUPS_STARTED="$GROUPS_STARTED $!"
wait_for "stereo_depth/compute" 60 bash -c "ros2 service list 2>/dev/null | grep -q stereo_depth/compute"
sleep 5
echo "== measuring: units $UNITS, $TARGETS look poses each, $RESTOCKS fillings per pose"
python3 "$HERE/pose_eval_robot.py" --out "$OUT" --units $UNITS --targets "$TARGETS" --restocks "$RESTOCKS"
code=$?
echo "== pose_eval_robot.py ended with code $code"
python3 "$HERE/pose_report.py" "$OUT"
exit "$code"
