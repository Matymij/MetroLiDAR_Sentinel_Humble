#!/bin/bash
# Прогон v21_FINAL по всем for_hackathon bag'ам.
cd "$(dirname "$0")/.."
source .venv/Scripts/activate

BAGS=(
  "doubleT_obstacle"
  "doubleT_platform"
  "roundT_doubleT"
  "roundT_pressureGate_roundT"
  "roundT_squareT_pressureGate_squareT"
  "squareT_platform_squareT_switch"
)

mkdir -p results/v21_FINAL/hackathon
for b in "${BAGS[@]}"; do
  bp="bags/for_hackathon/$b"
  if [ ! -f "$bp/metadata.yaml" ]; then
    echo "SKIP: $b (нет metadata)"
    continue
  fi
  echo ""
  echo "###### $b ######"
  python scripts/offline_pipeline_v23_FAST.py \
      "$bp" "results/v21_FINAL/hackathon/${b}.csv" 8 \
      2>&1 | tee "results/v21_FINAL/hackathon/${b}.log" | tail -20
done

echo ""
echo "===== СВОДКА ====="
for f in results/v21_FINAL/hackathon/*.csv; do
  [ -f "$f" ] || continue
  n=$(basename "$f" .csv)
  frames=$(awk 'END {print NR-1}' "$f")
  maxd=$(awk -F, 'NR>1 && $7>m {m=$7} END {print m}' "$f")
  echo "$n: $frames кадров, max=$maxd м"
done
