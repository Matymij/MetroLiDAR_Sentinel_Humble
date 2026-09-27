#!/bin/bash
set -euo pipefail
if [ $# -ne 1 ]; then echo "Usage: $0 /absolute/path/to/bag"; exit 2; fi
BAG="$1"
if [ ! -e "$BAG" ]; then echo "Bag not found: $BAG"; exit 3; fi
BAG_DIR="$(cd "$(dirname "$BAG")" && pwd)"; BAG_NAME="$(basename "$BAG")"
export BAG_DIR

docker compose run --rm sentinel bash -lc "ros2 launch /ws/launch/metro_sentinel.launch.py use_bag:=true bag:=/bags/$BAG_NAME"
