#!/usr/bin/env bash
# A series of independent attempts to pick an article with the true pose: the world is started again for every attempt, so
# that a jammed arm or a displaced base cannot spoil the next one.
#   usage: pick_series.sh [UNIT=S1] [ATTEMPTS=30] [OUT=/tmp/pick_series.json] [SEED=1]
# Run it in the container with nothing else running (pick_oracle.sh stops at once if a simulation is already up). Each
# attempt takes about 3 to 4 minutes, most of it to start the simulation, Nav2 is not used. When it ends, pick_report.py
# prints the counts.
UNIT=${1:-S1}
ATTEMPTS=${2:-30}
OUT=${3:-/tmp/pick_series.json}
SEED=${4:-1}
HERE=$(cd "$(dirname "$0")" && pwd)
DIR=$(mktemp -d /tmp/pick_series.XXXXXX)
echo "== series of $ATTEMPTS attempts at $UNIT, one world each; files in $DIR"

for i in $(seq 0 $((ATTEMPTS - 1))); do
  "$HERE/pick_oracle.sh" "$UNIT" 1 "$DIR/attempt_$i.json" $((SEED * 1000 + i)) > "$DIR/attempt_$i.log" 2>&1
  code=$?
  line=$(grep -E '^0: ' "$DIR/attempt_$i.log" | head -1)
  echo "attempt $i (exit $code): ${line:-no result, see $DIR/attempt_$i.log}"
  sleep 5
done

python3 - "$DIR" "$OUT" <<'PY'
import glob, json, os, sys
rows = []
for path in sorted(glob.glob(os.path.join(sys.argv[1], 'attempt_*.json')), key=lambda p: int(p.rsplit('_', 1)[1][:-5])):
    for r in json.load(open(path)):
        r['attempt'] = int(path.rsplit('_', 1)[1][:-5])
        rows.append(r)
json.dump(rows, open(sys.argv[2], 'w'), indent=1)
print(f'== {len(rows)} results in {sys.argv[2]}')
PY
python3 "$HERE/pick_report.py" "$OUT"
