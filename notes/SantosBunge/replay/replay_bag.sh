#!/usr/bin/env bash
# Replay a session/batch recording through deteccion + fusion and record what they publish.
#
#   replay_bag.sh <bag.mcap> <out_dir> [rate]
#
# Plays only the sensor topics, TF and the map (never the recorded /jar/* outputs), with the
# bag's clock; the nodes run with use_sim_time so their rate limit and TF lookups use the
# recorded time. Output: <out_dir>/replay/ (rosbag2 mcap with /jar/observaciones and
# /jar/candidatas) and <out_dir>/deteccion.log.
set -eo pipefail
BAG="$1"; OUT="$2"; RATE="${3:-1.0}"; WS="${4:-/jar_challenge}"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=66
mkdir -p "$OUT"
rm -rf "$OUT/replay"
# A YAML file wins over -p on the command line in Humble, so ablations pass a whole file.
PARAMS="${PARAMS_FILE_OVERRIDE:-$WS/src/equipo_jar/params/params.yaml}"

# Outputs are remapped to /replay/* so nothing the bag itself carries (its own /jar/* topics,
# latched or not) can mix with what the replayed nodes publish.
ros2 run equipo_jar deteccion --ros-args -p use_sim_time:=true --params-file "$PARAMS" \
  -r /jar/observaciones:=/replay/observaciones \
  --log-level deteccion:=${LOG_LEVEL:-info} > "$OUT/deteccion.log" 2>&1 &
DET=$!
ros2 run equipo_jar fusion --ros-args -p use_sim_time:=true --params-file "$PARAMS" \
  -r /jar/observaciones:=/replay/observaciones -r /jar/candidatas:=/replay/candidatas \
  > "$OUT/fusion.log" 2>&1 &
FUS=$!
ros2 bag record -s mcap -o "$OUT/replay" --use-sim-time /replay/observaciones /replay/candidatas \
  > "$OUT/record.log" 2>&1 &
REC=$!
sleep 3
ros2 bag play "$BAG" --clock 100 --rate "$RATE" \
  --topics /cam_1/color/image_raw /cam_1/color/camera_info /scan /cam_1/depth/color/points \
           /tf /tf_static /map \
  > "$OUT/play.log" 2>&1
sleep 3
# `ros2 run` is a wrapper: kill the node processes themselves, not only the wrappers,
# or they outlive the script and leak latched messages into the next replay.
kill -INT $REC; sleep 2
pkill -INT -f "equipo_jar/deteccion|equipo_jar/fusion|lib/equipo_jar" || true; sleep 2
pkill -9 -f "equipo_jar/deteccion|equipo_jar/fusion|lib/equipo_jar" || true
kill -9 $REC $DET $FUS 2>/dev/null || true
echo "replay done: $OUT/replay"
grep -E "last 10 s|Map received" "$OUT/deteccion.log" | tail -15
