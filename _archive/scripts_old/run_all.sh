#!/bin/bash
# Прогон v8 по всем bag'ам — логи на хосте, CSV из контейнера

BAGS=(
  "for_hackathon/doubleT_obstacle"
  "for_hackathon/doubleT_platform"
  "for_hackathon/roundT_doubleT"
  "for_hackathon/roundT_pressureGate_roundT"
  "for_hackathon/roundT_squareT_pressureGate_squareT"
  "for_hackathon/squareT_platform_squareT_switch"
)

mkdir -p results_v8

for bag in "${BAGS[@]}"; do
  name=$(echo "$bag" | tr '/' '_')
  echo ""
  echo "########## $bag ##########"

  # Копировать в /tmp контейнера для скорости
  docker exec sentinel_main bash -c "
    rm -rf /tmp/bag
    mkdir -p /tmp/bag
    cp /bags/${bag}/*.db3 /tmp/bag/ 2>/dev/null
    cp /bags/${bag}/metadata.yaml /tmp/bag/ 2>/dev/null
    ls -la /tmp/bag/
  "

  # Прогон — stdout/stderr пишем НА ХОСТЕ
  docker exec sentinel_main bash -c "
    source /opt/ros/humble/setup.bash
    source /ws/install/setup.bash
    python3 -u /ws/scripts/offline_pipeline.py /tmp/bag /tmp/out.csv 4
  " 2>&1 | tee "results_v8/${name}.log"

  # CSV из контейнера
  docker cp sentinel_main:/tmp/out.csv "results_v8/${name}.csv" 2>/dev/null

  # Метрики
  max_dist=$(grep "Max dist:" "results_v8/${name}.log" | awk '{print $3}')
  n_objs=$(grep "Всего уникальных" "results_v8/${name}.log" | awk '{print $NF}')
  echo "  → $n_objs объектов, max=$max_dist м"
done

echo ""
echo "===== ФИНАЛЬНАЯ СВОДКА ====="
for f in results_v8/*.csv; do
  name=$(basename "$f" .csv)
  if [ -f "$f" ]; then
    frames=$(awk 'END {print NR-1}' "$f")
    maxd=$(awk -F, 'NR>1 && $7>m {m=$7} END {print m}' "$f")
    echo "$name: $frames кадров, max=$maxd м"
  fi
done