#!/bin/bash
# Активация venv_gpu + LD_LIBRARY_PATH + symlinks для CUDA-библиотек из pip

PROJ_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJ_DIR"

source .venv_gpu/bin/activate

VENV_SITE="$PROJ_DIR/.venv_gpu/lib/python3.10/site-packages"
NV_ROOT="$VENV_SITE/nvidia"

# 1. Создаём symlink для всех lib*.so.X → lib*.so (numba_cuda ищет без версии)
if [ -d "$NV_ROOT" ]; then
    for d in "$NV_ROOT"/*/lib; do
        [ -d "$d" ] || continue
        for f in "$d"/*.so.*; do
            [ -f "$f" ] || continue
            base="${f%.so.*}.so"
            [ -e "$base" ] || ln -sf "$(basename "$f")" "$base"
        done
    done
fi

# 2. LD_LIBRARY_PATH
if [ -d "$NV_ROOT" ]; then
    for d in "$NV_ROOT"/*/lib; do
        [ -d "$d" ] && export LD_LIBRARY_PATH="$d:$LD_LIBRARY_PATH"
    done
fi

N=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -c nvidia || true)
echo "GPU env ready. nvidia dirs: $N"
