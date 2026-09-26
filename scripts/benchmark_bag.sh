#!/bin/bash
set -euo pipefail
if [ $# -ne 1 ]; then echo "Usage: $0 /absolute/path/to/bag"; exit 2; fi
BAG="$1"; BAG_DIR="$(cd "$(dirname "$BAG")" && pwd)"; BAG_NAME="$(basename "$BAG")"; export BAG_DIR
docker compose run --rm --network host sentinel bash -lc "ros2 run metro_benchmark benchmark_node --ros-args -p output_csv:=/benchmark/benchmark.csv & ros2 launch /ws/launch/metro_sentinel.launch.py use_bag:=true bag:=/bags/$BAG_NAME"
