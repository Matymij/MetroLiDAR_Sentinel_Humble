#!/bin/bash
# Прогон v11_FINAL по bag'ам с внешнего диска.
#
# Использование:
#   ./scripts/run_from_disk.sh <disk_bags_dir> [workers]
#
# Примеры:
#   ./scripts/run_from_disk.sh /d/bags 8
#   ./scripts/run_from_disk.sh "D:/MetroLiDAR_data" 8

set -e
cd "$(dirname "$0")/.."

DISK_DIR="$1"
WORKERS="${2:-8}"

if [ -z "$DISK_DIR" ]; then
  echo "Usage: $0 <disk_bags_dir> [workers]"
  echo "Example: $0 /d/bags 8"
  exit 1
fi

if [ ! -d "$DISK_DIR" ]; then
  echo "ERROR: directory not found: $DISK_DIR"
  exit 1
fi

# Найти все bag'ы (папки с metadata.yaml)
BAGS=$(find "$DISK_DIR" -maxdepth 3 -name "metadata.yaml" 2>/dev/null | \
       xargs -I{} dirname {} | sort -u)

if [ -z "$BAGS" ]; then
  echo "ERROR: не найдено ни одного bag'а в $DISK_DIR"
  echo "Ищу metadata.yaml на глубине до 3 уровней..."
  find "$DISK_DIR" -maxdepth 3 -name "metadata.yaml" 2>/dev/null
  exit 1
fi

echo "Найдено bag'ов: $(echo "$BAGS" | wc -l)"
echo "Рабочих: $WORKERS"
echo ""

mkdir -p results/disk_run
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG="results/disk_run/run_${TIMESTAMP}.log"

exec > >(tee -a "$LOG") 2>&1

for bag_path in $BAGS; do
  name=$(basename "$bag_path")
  echo ""
  echo "##############################################################"
  echo "##  $name"
  echo "##  $bag_path"
  echo "##############################################################"

  python scripts/offline_pipeline_v11_FINAL.py \
      "$bag_path" "results/disk_run/${name}.csv" "$WORKERS"
done

echo ""
echo "===== ИТОГОВАЯ СВОДКА ====="
for f in results/disk_run/*.csv; do
  [ -f "$f" ] || continue
  n=$(basename "$f" .csv)
  frames=$(awk 'END {print NR-1}' "$f")
  maxd=$(awk -F, 'NR>1 && $7>m {m=$7} END {print m}' "$f")
  echo "$n: $frames кадров, max=$maxd м"
done

echo ""
echo "Полный лог сохранён: $LOG"
