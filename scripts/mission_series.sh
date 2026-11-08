#!/usr/bin/env bash
# A series of missions, one world each (mission.sh restarts everything): the slot is drawn from the seed, the bin alternates.
#   ORDERS=2 mission_series.sh: missions of two articles
#   usage: mission_series.sh [MISSIONS=10] [OUT=/tmp/mission_series.json] [SEED=1] [UNIT=S1]
# Run it in the container with nothing else running. When it ends, the table of the missions is printed.
N=${1:-10}
OUT=${2:-/tmp/mission_series.json}
SEED=${3:-1}
UNIT=${4:-S1}
HERE=$(cd "$(dirname "$0")" && pwd)
DIR=$(mktemp -d /tmp/mission_series.XXXXXX)
echo "== series of $N missions at $UNIT, files in $DIR"
for i in $(seq 0 $((N - 1))); do
  S=$((SEED * 1000 + i))
  BINS=(bin_1 bin_2)
  rm -f /tmp/skills_trace.jsonl /tmp/mission_executor.log
  if [ "${ORDERS:-1}" = 1 ]; then BIN=${BINS[$((i % 2))]} ALT_BIN=${BINS[$(((i + 1) % 2))]}; fi        # two orders: the first to bin_1, the second to bin_2
  export BIN ALT_BIN
  "$HERE/mission.sh" "$UNIT" "$S" > "$DIR/mission_$i.log" 2>&1
  code=$?
  cp /tmp/skills_trace.jsonl "$DIR/trace_$i.jsonl" 2>/dev/null
  cp /tmp/mission_executor.log "$DIR/executor_$i.log" 2>/dev/null
  echo "mission $i (seed $S, ${BINS[$((i % 2))]}): exit $code: $(grep -E 'MISSION' "$DIR/mission_$i.log" | sed 's/.*\] //' | head -1)"
  sleep 5
done
python3 - "$DIR" "$OUT" <<'PY'
import glob, json, os, re, sys
rows = []
for path in sorted(glob.glob(os.path.join(sys.argv[1], 'mission_*.log')), key=lambda p: int(p.rsplit('_', 1)[1][:-4])):
    i = int(path.rsplit('_', 1)[1][:-4])
    text = open(path, errors='replace').read()
    trace = []
    try:
        trace = [json.loads(l) for l in open(os.path.join(sys.argv[1], f'trace_{i}.jsonl'))]
    except OSError:
        pass
    order = re.search(r'the (\S+) of (\S+) to (\S+)', text)
    orders = re.findall(r'the (\S+) of (\S+) to (bin_\d)', text)
    outcomes = dict(re.findall(r'\]: +(\w+_outcome|o\d_place) = (\S+)', text))
    skills = [t['skill'] for t in trace]
    rows.append({'mission': i, 'class': order and order.group(1), 'slot': order and order.group(2), 'bin': order and order.group(3), 'orders': orders,
                 'succeeded': 'MISSION SUCCEEDED' in text, 'seconds': (re.search(r'MISSION \w+ in ([\d.]+) s', text) or [None, None])[1],
                 'outcomes': outcomes, 'picks': skills.count('pick_article'), 'places': skills.count('place_in_bin'),
                 'article': next((t['detail'].get('article') for t in trace if t['skill'] == 'place_in_bin'), None)})
json.dump(rows, open(sys.argv[2], 'w'), indent=1)
print(f"== {sum(r['succeeded'] for r in rows)} of {len(rows)} missions succeeded; results in {sys.argv[2]}")
for r in rows:
    label = ' + '.join(c + ' ' + sl + ' ' + b for c, sl, b in r['orders']) if len(r['orders']) > 1 else f"{r['class']} {r['slot']} {r['bin']}"
    print(f"{r['mission']:3d} {label:60s} {'OK  ' if r['succeeded'] else 'FAIL'} {str(r['seconds']):>6s} s  picks {r['picks']}  places {r['places']}  {r['outcomes']}")
PY
