#!/usr/bin/env python3
"""Профиль CPU v9 на одном кадре — где реально тормозит."""
import sys, time
import numpy as np
sys.path.insert(0, "/ws/scripts")

# Импортируем v9 функции
from offline_pipeline import (
    rotate_axes, classify, fast_dbscan, cluster_layer,
    is_valid_object, _make_obj, process_frame, stream_frames,
    FWD_MIN, FWD_MAX, LAT_LIM, Z1, NEAR_MAX, MID_MAX, OVH_Z,
)

def main():
    bag = "/bags/cloud_with_fake_obj"
    print("читаем первый кадр...")
    for i, (ts, pts) in enumerate(stream_frames(bag)):
        break
    print(f"кадр: {pts.shape}\n")

    # 1. rotate_axes
    t0 = time.time()
    for _ in range(5):
        p = rotate_axes(pts.copy())
    print(f"rotate_axes: {(time.time()-t0)/5*1000:.1f} ms")

    # 2. ROI
    t0 = time.time()
    for _ in range(5):
        m = (p[:, 0] >= FWD_MIN) & (p[:, 0] <= FWD_MAX)
        m &= np.abs(p[:, 1]) <= LAT_LIM
        m &= p[:, 2] >= -0.3
        m &= p[:, 2] <= Z1 + 1.5
        roi = p[m]
    print(f"ROI filter: {(time.time()-t0)/5*1000:.1f} ms, roi={roi.shape[0]}")

    # 3. Слои
    x = roi[:, 0]
    near = roi[(x >= FWD_MIN) & (x < NEAR_MAX)]
    mid = roi[(x >= NEAR_MAX) & (x < MID_MAX)]
    far = roi[(x >= MID_MAX) & (x <= FWD_MAX)]
    print(f"near={near.shape[0]} mid={mid.shape[0]} far={far.shape[0]}\n")

    # 4. cluster_layer на каждом слое
    for name, chunk in (("near", near), ("mid", mid), ("far", far)):
        t0 = time.time()
        clusters = cluster_layer(chunk, name)
        dt = (time.time() - t0) * 1000
        total_pts = sum(c.shape[0] for c in clusters)
        print(f"cluster_layer({name}): {dt:.1f} ms, "
              f"{len(clusters)} кластеров, pts_in={chunk.shape[0]}, pts_out={total_pts}")

    # 5. Полный process_frame
    t0 = time.time()
    for _ in range(3):
        res = process_frame((ts, pts.copy()))
    print(f"\nprocess_frame (full): {(time.time()-t0)/3*1000:.1f} ms")


if __name__ == "__main__":
    main()