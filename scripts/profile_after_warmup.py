#!/usr/bin/env python3
"""Профиль pipeline после прогрева CUDA (100+ кадров)."""
import sys, time
import numpy as np
sys.path.insert(0, "/ws/scripts")
from offline_pipeline_gpu import stream_frames, _process_frame_impl
import cupy as cp

def main():
    bag = "/bags/cloud_with_fake_obj"
    print("прогрев CUDA и пайплайна на 50 кадрах...")
    t0 = time.time()
    for i, (ts, pts) in enumerate(stream_frames(bag)):
        res = _process_frame_impl(ts, pts)
        if i >= 50:
            break
    warmup = time.time() - t0
    print(f"прогрев 50 кадров: {warmup:.1f}s ({50/warmup:.1f} fps)\n")

    # Теперь измеряем следующие 100 кадров
    print("замер на 100 кадрах...")
    times = []
    frames = 0
    for i, (ts, pts) in enumerate(stream_frames(bag)):
        if i < 51:
            continue
        t0 = time.time()
        res = _process_frame_impl(ts, pts)
        times.append(time.time() - t0)
        frames += 1
        if frames >= 100:
            break

    times = np.array(times) * 1000
    print(f"100 кадров после прогрева:")
    print(f"  mean: {times.mean():.1f} ms ({1000/times.mean():.1f} fps)")
    print(f"  median: {np.median(times):.1f} ms")
    print(f"  min: {times.min():.1f} ms, max: {times.max():.1f} ms")
    print(f"  перцентили: p50={np.percentile(times,50):.0f} p90={np.percentile(times,90):.0f} p99={np.percentile(times,99):.0f}")

if __name__ == "__main__":
    main()