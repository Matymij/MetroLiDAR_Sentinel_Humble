#!/bin/bash
set -euo pipefail
docker compose run --rm --network host sentinel ros2 launch /ws/launch/metro_sentinel.launch.py
