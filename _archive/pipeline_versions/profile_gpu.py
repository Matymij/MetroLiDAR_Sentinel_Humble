#!/usr/bin/env python3
"""Профилировщик pipeline: где реально уходит время."""
import sys, time
from collections import Counter
import numpy as np

sys.path.insert(0, "scripts")
from offline_pipeline_OPT import (
    stream_frames, rotate_roi_numba, voxel_hash_numba,
    grid_dbscan_numba, _dbscan_ckdtree,
    fast_dbscan as fast_dbscan_cpu,
)

try:
    from offline_pipeline_GPU import _dbscan_cuml
    HAS_GPU = True
except Exception as e:
    print(f"GPU import failed: {e}")
    HAS_GPU = False

bag = sys.argv[1] if len(sys.argv) > 1 else "bags/cloud_with_fake_obj"

times = Counter()
counts = Counter()

for i, (ts, pts) in enumerate(stream_frames(bag)):
    if i >= 300:
        break
    t = time.perf_counter()
    roi = rotate_roi_numba(pts)
    times["rotate_roi"] += (time.perf_counter() - t) * 1000
    counts["frames"] += 1
    counts["pts_raw"] += pts.shape[0]
    counts["pts_roi"] += roi.shape[0]

    if roi.shape[0] == 0:
        continue

    near = roi[roi[:, 0] < 50.0]
    mid = roi[(roi[:, 0] >= 50.0) & (roi[:, 0] < 120.0)]
    far = roi[roi[:, 0] >= 120.0]

    for name, chunk, eps, minp, vox in [
        ("near", near, 0.50, 3, 0.10),
        ("mid", mid, 1.20, 3, 0.25),
        ("far", far, 4.00, 2, 0.35),
    ]:
        if chunk.shape[0] < minp:
            continue
        t = time.perf_counter()
        idx = voxel_hash_numba(chunk, vox)
        ch = chunk[idx]
        times[f"voxel_{name}"] += (time.perf_counter() - t) * 1000
        counts[f"n_{name}"] += ch.shape[0]

        t = time.perf_counter()
        _ = fast_dbscan_cpu(ch, eps, minp)
        times[f"dbscan_CPU_{name}"] += (time.perf_counter() - t) * 1000

        if HAS_GPU and ch.shape[0] >= 300:
            t = time.perf_counter()
            try:
                _ = _dbscan_cuml(ch, eps, minp)
            except Exception:
                pass
            times[f"dbscan_GPU_{name}"] += (time.perf_counter() - t) * 1000

n = max(counts['frames'], 1)
print()
print("===== ПРОФИЛЬ (300 кадров) =====")
print(f"frames: {counts['frames']}")
print(f"pts/frame raw: {counts['pts_raw']/n:.0f}")
print(f"pts/frame roi: {counts['pts_roi']/n:.0f}")
print()
print("Средние размеры облака после voxel:")
for lname in ("near", "mid", "far"):
    c = counts.get(f"n_{lname}", 0)
    print(f"  {lname}: {c/n:.0f} точек")
print()
print("Время на кадр (мс):")
for k in sorted(times.keys()):
    v = times[k] / n
    print(f"  {k:22s} {v:7.2f} ms")
