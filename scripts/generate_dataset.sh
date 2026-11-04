#!/usr/bin/env bash
# Generate the dataset with the free camera, restarting the world between batches.
#   usage: generate_dataset.sh OUT [ARRANGEMENTS=200] [BATCH=40] [SHOTS=20]
# Run it in the container, with nothing else running: it stops at once if a simulation is already up. A batch ends when
# BATCH arrangements are done or when the generator asks for a new world (exit code 75); the script goes on until
# ARRANGEMENTS have been saved in OUT, also over several runs.
OUT=${1:?usage: generate_dataset.sh OUT [ARRANGEMENTS] [BATCH] [SHOTS]}
TOTAL=${2:-200}
BATCH=${3:-40}
SHOTS=${4:-20}
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
HERE=$(cd "$(dirname "$0")" && pwd)

BUSY=$(ps -eo pid,args | grep -E "ign gazebo|move_group|parameter_bridge|robot_state_publisher|component_container|stereo_depth_node|ros2 launch|follow_joint_trajectory" | grep -v grep | grep -v "bash -l")
if [ -n "$BUSY" ]; then echo "something is already running, nothing started:"; echo "$BUSY" | cut -c1-100; exit 3; fi

WORLD_PID=""
OWN=""
start_world() {
  ros2 launch amr_manipulator_gazebo dataset.launch.py > /tmp/dataset_world.log 2>&1 &
  WORLD_PID=$!
  for _ in $(seq 60); do                       # wait until the world answers
    sleep 2
    ign topic -e -t /world/amr_world/stats -n 1 2>/dev/null | grep -q real_time_factor && break
  done
  sleep 10                                     # the camera and the bridge come up after the world
  OWN=$(ps -eo pid,args | grep -E "ign gazebo|parameter_bridge|ros2 launch|ros_gz_sim" | grep -v grep | awk '{print $1}' | tr '\n' ' ')
}
stop_world() {
  [ -n "$WORLD_PID" ] && kill -TERM "$WORLD_PID" 2>/dev/null
  sleep 4
  [ -n "$OWN" ] && kill -9 $OWN 2>/dev/null
  WORLD_PID=""; OWN=""
}
trap stop_world EXIT

count_done() {                                 # arrangements saved so far, from the stats files of all the batches
  python3 - "$OUT" <<'PY'
import glob, json, sys
print(sum(sum(v['groups'] for v in json.load(open(f))['splits'].values()) for f in glob.glob(sys.argv[1] + '/stats_seed*.json')))
PY
}

stalled=0
while :; do
  done_now=$(count_done)
  left=$((TOTAL - done_now))
  [ "$left" -le 0 ] && break
  n=$(( left < BATCH ? left : BATCH ))
  seed=$(( $(ls "$OUT"/stats_seed*.json 2>/dev/null | wc -l) + 1 ))
  echo "== batch with seed $seed: $n arrangements ($done_now of $TOTAL done)"
  start_world
  python3 "$HERE/make_dataset_free.py" --out "$OUT" --arrangements "$n" --shots "$SHOTS" --seed "$seed"
  code=$?
  stop_world
  if [ "$(count_done)" -eq "$done_now" ]; then stalled=$((stalled + 1)); else stalled=0; fi
  if [ "$stalled" -ge 2 ]; then echo "two batches in a row without a saved arrangement: stopping"; exit 1; fi
  if [ "$code" -ne 0 ] && [ "$code" -ne 75 ]; then echo "the generator failed with code $code"; exit "$code"; fi
done
echo "== done: $(count_done) arrangements in $OUT"
