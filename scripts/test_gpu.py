#!/usr/bin/env python3
"""Тест GPU: cupy + бенчмарк voxel vs numpy."""
import time
import numpy as np

try:
    import cupy as cp
    HAS_CUPY = True
except Exception as e:
    HAS_CUPY = False
    print(f"cupy недоступен: {e}")


def main():
    if not HAS_CUPY:
        return
    print(f"cupy version: {cp.__version__}")
    print(f"CUDA devices: {cp.cuda.runtime.getDeviceCount()}")
    a = cp.array([1, 2, 3, 4, 5])
    b = a * 2
    print(f"GPU test: {b.get()}")
    z = cp.arange(1000000)
    print(f"1M sum: {int(z.sum())}")
    name = cp.cuda.runtime.getDeviceProperties(0)['name']
    if isinstance(name, bytes):
        name = name.decode()
    print(f"Device: {name}")

    # Бенчмарк
    n = 307200
    pts_np = np.random.randn(n, 3).astype(np.float32)
    pts_gpu = cp.asarray(pts_np)

    def voxel_np(pts, vox=0.05):
        keys = np.floor(pts / vox).astype(np.int64)
        _, idx = np.unique(keys, axis=0, return_index=True)
        return pts[np.sort(idx)]

    def voxel_gpu(pts, vox=0.05):
        keys = cp.floor(pts / vox).astype(cp.int64)
        _, idx = cp.unique(keys, axis=0, return_index=True)
        return pts[cp.sort(idx)]

    for _ in range(3):
        _ = voxel_gpu(pts_gpu)
    cp.cuda.Stream.null.synchronize()

    t0 = time.time()
    for _ in range(10):
        r_np = voxel_np(pts_np)
    t_cpu = (time.time() - t0) / 10

    t0 = time.time()
    for _ in range(10):
        r_gpu = voxel_gpu(pts_gpu)
        cp.cuda.Stream.null.synchronize()
    t_gpu = (time.time() - t0) / 10

    print()
    print(f"CPU voxel: {t_cpu*1000:.1f} ms")
    print(f"GPU voxel: {t_gpu*1000:.1f} ms")
    print(f"Speedup: {t_cpu/t_gpu:.1f}x")
    print(f"Points: CPU={r_np.shape[0]}, GPU={r_gpu.get().shape[0]}")


if __name__ == "__main__":
    main()