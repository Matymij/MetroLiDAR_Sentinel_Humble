#!/bin/bash
# Быстрый просмотр содержимого внешнего диска: какие bag'и, сколько кадров, размер.
#
# Использование: ./scripts/inspect_disk.sh <disk_dir>

DISK_DIR="$1"
if [ -z "$DISK_DIR" ] || [ ! -d "$DISK_DIR" ]; then
  echo "Usage: $0 <disk_dir>"
  exit 1
fi

echo "=== Структура: $DISK_DIR (глубина 2) ==="
find "$DISK_DIR" -maxdepth 2 -type d | head -30
echo ""
echo "=== Размер ==="
du -sh "$DISK_DIR" 2>/dev/null
echo ""
echo "=== Bag'и (папки с metadata.yaml) ==="
find "$DISK_DIR" -maxdepth 3 -name "metadata.yaml" 2>/dev/null | while read m; do
  d=$(dirname "$m")
  sz=$(du -sh "$d" 2>/dev/null | cut -f1)
  db3=$(find "$d" -name "*.db3" | head -1)
  fs=$(du -h "$db3" 2>/dev/null | cut -f1)
  echo "  $d  ($sz, db3=$fs)"
done
echo ""
echo "=== Что за топики в первом bag'е ==="
FIRST_BAG=$(find "$DISK_DIR" -maxdepth 3 -name "metadata.yaml" 2>/dev/null | head -1 | xargs dirname)
if [ -n "$FIRST_BAG" ]; then
  python - << PYEOF
from rosbags.rosbag2 import Reader
try:
    with Reader("$FIRST_BAG") as r:
        for c in r.connections:
            print(f"  {c.topic:40s}  {c.msgtype:45s}  {c.msgcount:6d} msgs")
except Exception as e:
    print(f"  ERROR: {e}")
PYEOF
fi
