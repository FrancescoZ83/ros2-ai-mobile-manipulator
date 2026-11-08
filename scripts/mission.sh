#!/usr/bin/env bash
# A mission run by the behavior tree: fetch an article from a slot of a shelf unit and put it in a bin of the packing table.
#   usage: [SLOT=S1.L2.n1] [BIN=bin_1] [ALT_BIN=bin_2] [TREE=fetch_and_deliver.xml] [CLASSES="water_bottle"] [ARTICLE_CLASS=...] mission.sh [UNIT=S1] [SEED=1]
#   ORDERS=2: two articles (slots drawn from SEED), the first to BIN (bin_1), the second to BIN2 (bin_2), tree fetch_two.xml
#   GUI=true: with the window of Gazebo (a display is needed), to watch the robot; the default is without
#   ARTICLE_CLASS: ask the tree for another class than the one in the slot (a failure to try the retries); BIN=bin_9: a bin that does not exist
# Starts the simulation (the robot at the origin), Nav2 (tight AMCL), move_group and the stereo node, stocks the shelves and picks
# the ordered article (mission_setup.py: the slot drawn from SEED unless SLOT is given), starts the skill server (skill_server.py) and
# runs mission_executor on the tree. Exit code of mission_executor: 0 the mission succeeded, 1 it failed.
# Run it in the container with nothing else running; it stops only what it started.
UNIT=${1:-S1}
SEED=${2:-1}
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
HERE=$(cd "$(dirname "$0")" && pwd)

BUSY=$(ps -eo pid,args | grep -E "ign gazebo|move_group|parameter_bridge|robot_state_publisher|component_container|stereo_depth_node|ros2 launch|follow_joint_trajectory|skill_server" | grep -v grep | grep -v "bash -l")
if [ -n "$BUSY" ]; then echo "something is already running, nothing started:"; echo "$BUSY" | cut -c1-100; exit 3; fi

STARTED=""
OWN=""
start() {                                        # start NAME COMMAND...: a process group of its own
  local name=$1; shift
  setsid "$@" > "/tmp/mission_$name.log" 2>&1 &
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
  echo "timeout waiting for $what (logs in /tmp/mission_*.log)"; exit 4
}
world_up() { ign topic -e -t /world/amr_world/stats -n 1 2>/dev/null | grep -q real_time_factor; }
has_action() { ros2 action list 2>/dev/null | grep -q "$1"; }

echo "== simulation"
start sim ros2 launch amr_manipulator_gazebo sim.launch.py world:=warehouse.sdf gui:=${GUI:-false} stereo:=true dataset:=true
wait_for "the world" 120 world_up
OWN=$(ps -eo pid,args | grep -E "ign gazebo|ruby.*ign" | grep -v grep | awk '{print $1}' | tr '\n' ' ')
wait_for "follow_joint_trajectory" 90 has_action follow_joint_trajectory
sleep 30                                         # the arm goes to the travel pose after the start
echo "== Nav2"
start nav ros2 launch amr_manipulator_gazebo nav.launch.py params:=$(ros2 pkg prefix amr_manipulator_gazebo)/share/amr_manipulator_gazebo/config/nav2_params_precise.yaml
wait_for "navigate_to_pose" 90 has_action navigate_to_pose
sleep 20
echo "== MoveIt"
start moveit ros2 launch amr_manipulator_moveit_config move_group.launch.py
wait_for "move_action" 90 has_action move_action
sleep 15
start stereo ros2 run amr_manipulator_perception stereo_depth_node.py
wait_for "stereo_depth/compute" 60 bash -c "ros2 service list 2>/dev/null | grep -q stereo_depth/compute"
sleep 3

echo "== stock and order"
ORDER=$(cd "$HERE" && python3 mission_setup.py --unit "$UNIT" --seed "$SEED" ${SLOT:+--slot $SLOT} ${CLASSES:+--classes $CLASSES} --orders ${ORDERS:-1} 2>/dev/null | tail -1)
echo "$ORDER"
read -r ORDER_SLOT ORDER_CLASS <<< "$(python3 -c "import json,sys; o=json.loads(sys.argv[1]); print(o['slot'], o['article_class'])" "$ORDER")"
[ -z "$ORDER_SLOT" ] && { echo "no order"; exit 5; }

echo "== skill server"
start skills python3 "$HERE/skill_server.py"
wait_for "the skill server" 120 bash -c "ros2 action list 2>/dev/null | grep -q skills/place_in_bin"
sleep 5

if [ "${ORDERS:-1}" = 1 ]; then
  echo "== mission: the $ORDER_CLASS of $ORDER_SLOT to ${BIN:-bin_1}"
  ros2 run amr_manipulator_task mission_executor --ros-args -p slot:="$ORDER_SLOT" -p article_class:="${ARTICLE_CLASS:-$ORDER_CLASS}" \
    -p bin:="${BIN:-bin_1}" -p alt_bin:="${ALT_BIN:-bin_2}" -p tree:="${TREE:-fetch_and_deliver.xml}" 2>&1 | tee /tmp/mission_executor.log
else                                             # ORDERS=2: two articles, the first to bin_1 and the second to bin_2 (the other one is the one to try if it fails)
  read -r S1 C1 S2 C2 <<< "$(python3 -c "import json,sys; o=json.loads(sys.argv[1])['orders']; print(o[0]['slot'], o[0]['article_class'], o[1]['slot'], o[1]['article_class'])" "$ORDER")"
  echo "== mission: the $C1 of $S1 to ${BIN:-bin_1} and the $C2 of $S2 to ${BIN2:-bin_2}"
  ros2 run amr_manipulator_task mission_executor --ros-args -p tree:="${TREE:-fetch_two.xml}" \
    -p o1_slot:="$S1" -p o1_class:="${ARTICLE_CLASS:-$C1}" -p o1_bin:="${BIN:-bin_1}" -p o1_alt:="${BIN2:-bin_2}" \
    -p o2_slot:="$S2" -p o2_class:="$C2" -p o2_bin:="${BIN2:-bin_2}" -p o2_alt:="${BIN:-bin_1}" 2>&1 | tee /tmp/mission_executor.log
fi
code=${PIPESTATUS[0]}
echo "== mission_executor ended with code $code"
exit "$code"
