#!/usr/bin/env python3
"""Профиль v12: где теряем время. Запускать в контейнере."""
import sys
import time
import numpy as np

sys.path.insert(0, "/ws/scripts")

from offline_pipeline_FINAL import (
    rotate_roi_numba, voxel_hash_numba, grid_dbscan_numba,
    cluster_layer, classify, is_valid_object, Tracker,
    stream_frames, NEAR_MAX, MID_MAX,
)


def timeit(name, fn, n=20):
    for _ in range(3):
        fn()
    t0 = time.time()
    for _ in range(n):
        fn()
    dt = (time.time() - t0) / n * 1000
    print(f"  {name:35s}: {dt:7.2f} ms")
    return dt


def main():
    bag = "/tmp/bag"
    print(f"читаем 1 кадр из {bag}...")
    for i, (ts, pts) in enumerate(stream_frames(bag)):
        break
    print(f"кадр: {pts.shape}\n")

    # 1. rotate_roi
    timeit("rotate_roi_numba (307K)", lambda: rotate_roi_numba(pts))

    roi = rotate_roi_numba(pts)
    print(f"  roi: {roi.shape[0]} точек")

    x = roi[:, 0]
    near = roi[(x >= 0.3) & (x < NEAR_MAX)]
    mid = roi[(x >= NEAR_MAX) & (x < MID_MAX)]
    far = roi[(x >= MID_MAX) & (x <= 230.0)]
    print(f"  Слои: near={near.shape[0]} mid={mid.shape[0]} far={far.shape[0]}")

    # 2. Кластеризация
    for layer, chunk in (("near", near), ("mid", mid), ("far", far)):
        t0 = time.time()
        for _ in range(5):
            clusters = cluster_layer(chunk, layer)
        dt = (time.time() - t0) / 5 * 1000
        print(f"  cluster_layer({layer:5s}): {dt:7.2f} ms ({len(clusters)} кл.)")

    # 3. classify+filter
    clusters = cluster_layer(near, "near")
    t0 = time.time()
    for _ in range(20):
        for mn, mx, npts in clusters:
            cy = float((mn[1] + mx[1]) / 2)
            cz = float((mn[2] + mx[2]) / 2)
            dist = float(max(0.0, (mn[0] + mx[0]) / 2))
            zone = classify(cy, cz)
            ok, why = is_valid_object(mn, mx, zone, dist, npts)
    dt = (time.time() - t0) / 20 * 1000
    print(f"  classify+filter (near):       {dt:7.2f} ms ({len(clusters)} кл.)")

    # 4. Tracker
    tracker = Tracker(assoc_dist=6.0, timeout=8, min_hits=15)
    dets = [
        {"cx": 5.0 + i*0.1, "cy": 0.5, "cz": 1.0, "dist": 5.0,
         "sx": 0.3, "sy": 0.3, "sz": 0.3, "zone": "INSIDE", "npts": 10}
        for i in range(20)
    ]
    t0 = time.time()
    for i in range(100):
        tracker.update(dets, float(i))
    dt = (time.time() - t0) / 100 * 1000
    print(f"  Tracker.update (20 dets):     {dt:7.2f} ms")

    # 5. I/O
    t0 = time.time()
    n = 0
    for i, (ts, p) in enumerate(stream_frames(bag)):
        n += 1
        if n >= 20:
            break
    dt = (time.time() - t0) / 20 * 1000
    print(f"\n  I/O: чтение кадра:            {dt:7.2f} ms")


if __name__ == "__main__":
    main()