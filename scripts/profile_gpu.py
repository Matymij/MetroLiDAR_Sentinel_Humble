#!/usr/bin/env python3
"""Профиль GPU pipeline: где уходит время в _process_frame_impl."""
import time
import numpy as np
import cupy as cp
import sys
sys.path.insert(0, "/ws/scripts")
from offline_pipeline_gpu import (rotate_axes_gpu, voxel_gpu, preprocess_gpu,
                                  cluster_layer, classify_cpu, is_valid_object,
                                  NEAR_MAX, MID_MAX, FWD_MIN, FWD_MAX, LAT_LIM,
                                  Z1)


def timeit(label, fn, n=10):
    # warmup
    for _ in range(2):
        fn()
    cp.cuda.Stream.null.synchronize()
    t0 = time.time()
    for _ in range(n):
        fn()
        cp.cuda.Stream.null.synchronize()
    dt = (time.time() - t0) / n * 1000
    print(f"  {label:30s}: {dt:7.1f} ms")
    return dt


def main():
    n = 307200
    pts_np = np.random.randn(n, 3).astype(np.float32)
    pts_np[:, 0] = np.abs(pts_np[:, 0]) * 20  # forward
    pts_np[:, 1] = np.random.uniform(-3, 3, n)  # lateral
    pts_np[:, 2] = np.random.uniform(-1, 4, n)  # z
    pts_gpu = cp.asarray(pts_np)

    print(f"Профиль: {n} точек, 10 итераций каждое")

    # 1. Transfer CPU → GPU
    timeit("cp.asarray (307K)", lambda: cp.asarray(pts_np))

    # 2. Rotate
    timeit("rotate_axes_gpu", lambda: rotate_axes_gpu(pts_gpu.copy()))

    # 3. Voxel на GPU (только 30K случайных точек — как реальный near слой)
    pts_small = cp.asarray(np.random.randn(50000, 3).astype(np.float32))
    timeit("voxel_gpu (50K)", lambda: voxel_gpu(pts_small, 0.04))
    timeit("voxel_gpu (150K)", lambda: voxel_gpu(cp.asarray(np.random.randn(150000, 3).astype(np.float32)), 0.15))

    # 4. Voxel на CPU для сравнения
    from offline_pipeline import cluster_layer as cluster_cpu
    pts_small_np = np.random.randn(50000, 3).astype(np.float32)
    def voxel_np(pts, vox=0.04):
        keys = np.floor(pts / vox).astype(np.int64)
        _, idx = np.unique(keys, axis=0, return_index=True)
        return pts[np.sort(idx)]
    timeit("voxel_numpy (50K)", lambda: voxel_np(pts_small_np))

    # 5. Transfer GPU → CPU
    timeit("cp.asnumpy (50K)", lambda: cp.asnumpy(pts_small))

    # 6. ROI filter GPU
    def roi_gpu():
        p = pts_gpu.copy()
        m = (p[:, 0] >= FWD_MIN) & (p[:, 0] <= FWD_MAX)
        m &= cp.abs(p[:, 1]) <= LAT_LIM
        m &= p[:, 2] >= -0.3
        m &= p[:, 2] <= Z1 + 1.5
        return p[m]
    timeit("ROI filter (307K)", roi_gpu)


if __name__ == "__main__":
    main()