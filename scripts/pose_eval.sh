#!/usr/bin/env bash
# Measure the pose estimate of the articles against the true pose, in the world with the free camera.
#   usage: [DUMP=frame.npz] [EXTRA='--only-class soup_can'] pose_eval.sh OUT.json [ARRANGEMENTS=30] [SHOTS=5] [NETWORK=base]   (DUMP: save the first view)
# Run it in the container with nothing else running: it stops at once if a simulation is already up. It starts the world
# (dataset.launch.py) and the stereo node, runs pose_eval.py, prints the report and stops only what it started.
OUT=${1:?usage: pose_eval.sh OUT.json [ARRANGEMENTS] [SHOTS] [NETWORK]}
ARRANGEMENTS=${2:-30}
SHOTS=${3:-5}
NETWORK=${4:-base}
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
HERE=$(cd "$(dirname "$0")" && pwd)

BUSY=$(ps -eo pid,args | grep -E "ign gazebo|move_group|parameter_bridge|robot_state_publisher|component_container|stereo_depth_node|ros2 launch|follow_joint_trajectory" | grep -v grep | grep -v "bash -l")
if [ -n "$BUSY" ]; then echo "something is already running, nothing started:"; echo "$BUSY" | cut -c1-100; exit 3; fi

STARTED=""
start() {                                        # start NAME COMMAND...: a process group of its own
  local name=$1; shift
  setsid "$@" > "/tmp/pose_eval_$name.log" 2>&1 &
  STARTED="$STARTED $!"
}
stop_all() {
  for pid in $STARTED; do kill -TERM -- "-$pid" 2>/dev/null; done
  sleep 5
  for pid in $STARTED; do kill -KILL -- "-$pid" 2>/dev/null; done
  STARTED=""
}
trap stop_all EXIT

start world ros2 launch amr_manipulator_gazebo dataset.launch.py
for _ in $(seq 60); do
  sleep 2
  ign topic -e -t /world/amr_world/stats -n 1 2>/dev/null | grep -q real_time_factor && break
done
sleep 10
start stereo ros2 run amr_manipulator_perception stereo_depth_node.py
for _ in $(seq 30); do
  sleep 2
  ros2 service list 2>/dev/null | grep -q stereo_depth/compute && break
done
sleep 3
python3 "$HERE/pose_eval.py" --out "$OUT" --arrangements "$ARRANGEMENTS" --shots "$SHOTS" --name "$NETWORK" ${DUMP:+--dump "$DUMP"} ${EXTRA:-}
code=$?
echo "== pose_eval.py ended with code $code"
python3 "$HERE/pose_report.py" "$OUT"
exit "$code"
