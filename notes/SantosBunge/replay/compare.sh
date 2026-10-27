#!/usr/bin/env bash
# compare.sh <map_name> <bag_dir> <tag> [extra ros params for the "new" run, e.g. "-p cloud_check:=false"]
# Replays the bag through the baseline (development) and the new code, each in its own
# container, then evaluates both against the practice world's boxes.
set -eo pipefail
MAP="$1"; BAGDIR="$2"; TAG="$3"; EXTRA="${4:-}"
S="${REPLAY_DIR:-$HOME/jar_replay}"      # where replays and logs go
REPO="${REPO:-$(cd "$(dirname "$0")/../../.." && pwd)}"   # this checkout (the "new" code)
run() {  # run <ws_mount_dir> <ws_in_container> <out>
  rm -rf "$S/replay/$3"
  OVR=""
  if [ -n "$EXTRA" ]; then
    # EXTRA = "key=value key=value" applied to the deteccion section of params.yaml
    python3 - "$REPO/src/equipo_jar/params/params.yaml" "$S/replay/$3_params.yaml" $EXTRA <<'PY'
import re, sys
src, dst, *pairs = sys.argv[1:]
text = open(src).read()
for kv in pairs:
    k, v = kv.split('=')
    text, n = re.subn(rf'^(\s+{k}:\s*)\S+', rf'\g<1>{v}', text, count=1, flags=re.M)
    assert n == 1, k
open(dst, 'w').write(text)
PY
    OVR="-e PARAMS_FILE_OVERRIDE=/scratch/replay/$3_params.yaml"
  fi
  docker run --rm -v "$1":"$2" -v "$BAGDIR":/bag:ro -v "$S":/scratch $OVR -e LOG_LEVEL="${LOG_LEVEL:-info}" jar-dev \
    bash -c "/scratch/replay_bag.sh /bag/grabacion_0.mcap /scratch/replay/$3 1.0 $2" > "$S/replay/$3.log" 2>&1 || true
}
if [ -z "$EXTRA" ]; then run "${BASELINE:-$S/baseline}" /baseline "${TAG}_base"; fi
run "$REPO" /jar_challenge "${TAG}_new"
docker run --rm -v "$REPO":/jar_challenge -v "$BAGDIR":/bag:ro -v "$S":/scratch jar-dev bash -c "
  source /opt/ros/humble/setup.bash && source /jar_challenge/install/setup.bash
  [ -d /scratch/replay/${TAG}_base/replay ] && python3 /scratch/eval_replay.py $MAP /bag/grabacion_0.mcap /scratch/replay/${TAG}_base/replay baseline
  python3 /scratch/eval_replay.py $MAP /bag/grabacion_0.mcap /scratch/replay/${TAG}_new/replay 'new $EXTRA'
  echo '=== rejections (new) ==='; grep -h 'last 10 s' /scratch/replay/${TAG}_new/deteccion.log | sed -E 's/.*last 10 s: //'
" 2>&1 | tail -45
