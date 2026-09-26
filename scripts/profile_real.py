#!/usr/bin/env python3
"""Профиль GPU pipeline на первом кадре bag."""
import sys
import time
import numpy as np
import cupy as cp
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, "/ws/scripts")
from offline_pipeline_gpu import (preprocess_gpu, cluster_layer,
                                   classify_cpu, is_valid_object,
                                   rotate_axes_gpu, voxel_gpu)


def main():
    bag = "/bags/cloud_with_fake_obj"
    print(f"открываем {bag}...")
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    cloud = next(n for n, t in topics.items()
                 if t == "sensor_msgs/msg/PointCloud2")

    # Первый кадр
    topic, data, ts = reader.read_next()
    while topic != cloud:
        topic, data, ts = reader.read_next()

    msg = deserialize_message(data, PointCloud2)
    n, ps = msg.width * msg.height, msg.point_step
    buf = np.frombuffer(msg.data, dtype=np.uint8, count=n * ps)
    pts = buf.reshape(n, ps)[:, :12].view(np.float32).reshape(n, 3).copy()
    print(f"кадр: {pts.shape}, point_step={ps}\n")

    # === 1. cp.asarray ===
    t0 = time.time()
    for _ in range(3):
        pts_gpu = cp.asarray(pts)
        cp.cuda.Stream.null.synchronize()
    print(f"cp.asarray (307K): {(time.time()-t0)/3*1000:.1f} ms")

    # === 2. rotate_gpu ===
    t0 = time.time()
    for _ in range(3):
        _ = rotate_axes_gpu(pts_gpu.copy())
        cp.cuda.Stream.null.synchronize()
    print(f"rotate_axes_gpu: {(time.time()-t0)/3*1000:.1f} ms")

    # === 3. Только ROI (без voxel) ===
    def roi_only():
        p = pts_gpu.copy()
        m = (p[:, 0] >= 0.3) & (p[:, 0] <= 230.0)
        m &= cp.abs(p[:, 1]) <= 3.0
        m &= p[:, 2] >= -0.3
        m &= p[:, 2] <= 4.9
        return p[m]
    t0 = time.time()
    for _ in range(3):
        roi = roi_only()
        cp.cuda.Stream.null.synchronize()
    print(f"ROI только: {(time.time()-t0)/3*1000:.1f} ms, roi_pts={roi.shape[0]}")

    # === 4. voxel каждого слоя ===
    x = roi[:, 0]
    near = roi[(x >= 0.3) & (x < 50)]
    mid = roi[(x >= 50) & (x < 120)]
    far = roi[(x >= 120) & (x <= 230)]
    print(f"near={near.shape[0]} mid={mid.shape[0]} far={far.shape[0]}\n")

    for name, chunk, vox in (("near", near, 0.04), ("mid", mid, 0.15), ("far", far, 0.20)):
        t0 = time.time()
        for _ in range(3):
            _ = voxel_gpu(chunk, vox)
            cp.cuda.Stream.null.synchronize()
        dt = (time.time()-t0)/3*1000
        print(f"voxel_gpu({name}, {chunk.shape[0]}pts, vox={vox}): {dt:.1f} ms")

    # === 5. transfer GPU → CPU для каждого слоя ===
    near_v = cp.asnumpy(voxel_gpu(near, 0.04)).astype(np.float32)
    mid_v = cp.asnumpy(voxel_gpu(mid, 0.15)).astype(np.float32)
    far_v = cp.asnumpy(voxel_gpu(far, 0.20)).astype(np.float32)
    print(f"\nпосле voxel: near={near_v.shape[0]} mid={mid_v.shape[0]} far={far_v.shape[0]}")

    # === 6. cluster_layer каждый слой ===
    for name, chunk in (("near", near_v), ("mid", mid_v), ("far", far_v)):
        t0 = time.time()
        clusters = cluster_layer(chunk, name)
        dt = (time.time() - t0) * 1000
        total_pts = sum(c.shape[0] for c in clusters)
        print(f"cluster_layer({name}): {dt:.1f} ms, "
              f"{len(clusters)} кластеров, pts={total_pts}")

    print("\n=== ИТОГО ===")
    t0 = time.time()
    near_v2 = cp.asnumpy(voxel_gpu(cp.asarray(pts, dtype=cp.float32) if False else near, 0.04)).astype(np.float32)
    print(f"Полный pipeline одного кадра: {time.time()-t0:.3f} сек")


if __name__ == "__main__":
    main()