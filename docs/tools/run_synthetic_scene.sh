#!/bin/bash
# Runs every documentation scenario through the real surface_detector and approach_planner, each with fresh
# nodes on an isolated ROS domain. Usage: run_synthetic_scene.sh <output_dir>   (source the workspace first)
set -u
OUT=${1:?output directory}
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$OUT"
export ROS_DOMAIN_ID=${SCENE_DOMAIN_ID:-88} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST PYTHONDONTWRITEBYTECODE=1

run() {
    local scenario=$1 frames=$2 wait_s=$3 duration=$4
    timeout -s INT "$duration" ros2 launch surface_detection surface_detector.launch.py > "$OUT/surface_$scenario.log" 2>&1 &
    timeout -s INT "$duration" ros2 launch approach_path_planning approach_planner.launch.py > "$OUT/planner_$scenario.log" 2>&1 &
    sleep 3
    FRAMES=$frames WAIT=$wait_s timeout $((duration - 4)) python3 "$HERE/synthetic_scene.py" "$OUT/scene_$scenario.json" "$scenario" > /dev/null 2>&1
    wait
    echo "$scenario: goals=$(grep -c 'Found goal' "$OUT/planner_$scenario.log") no_goal=$(grep -c 'Unable to find' "$OUT/planner_$scenario.log")"
}

run baseline 5 1.2 24
run straight 5 1.2 24
run blocked 5 1.2 24
run two_sided 5 1.2 24
run too_far 5 1.2 24
# This frame takes ~10 s with thicken_ransac enabled (see docs/surface_detection.md)
run fov_clipped 1 13 24
