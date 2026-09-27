#!/bin/bash
BASE="/tmp/for_hackathon/for_hackathon"
BAGS=(
  "doubleT_obstacle"
  "doubleT_platform"
  "roundT_doubleT"
  "roundT_pressureGate_roundT"
  "roundT_squareT_pressureGate_squareT"
  "squareT_platform_squareT_switch"
)

mkdir -p results_v9

for bag in "${BAGS[@]}"; do
  echo ""
  echo "########## $bag ##########"

  docker exec sentinel_main bash -c "
    source /opt/ros/humble/setup.bash
    source /ws/install/setup.bash
    python3 -u /ws/scripts/offline_pipeline.py ${BASE}/$bag /tmp/out_$bag.csv 4
  " 2>&1 | tee "results_v9/${bag}.log"

  docker cp sentinel_main:/tmp/out_${bag}.csv "results_v9/${bag}.csv" 2>/dev/null

  max_dist=$(grep "Max dist:" "results_v9/${bag}.log" | awk '{print $3}')
  n_objs=$(grep "Всего уникальных" "results_v9/${bag}.log" | awk '{print $NF}')
  echo "  → $n_objs объектов, max=$max_dist м"
done

echo ""
echo "===== ФИНАЛЬНАЯ СВОДКА v9 ====="
for f in results_v9/*.csv; do
  name=$(basename "$f" .csv)
  if [ -f "$f" ]; then
    frames=$(awk 'END {print NR-1}' "$f")
    maxd=$(awk -F, 'NR>1 && $7>m {m=$7} END {print m}' "$f")
    echo "$name: $frames кадров, max=$maxd м"
  fi
done