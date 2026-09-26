#!/usr/bin/env python3
"""Offline pipeline GPU — cupy для preprocess, CPU для DBSCAN + tracking.

Основное ускорение:
- Voxel downsampling: 20×
- ROI-фильтр: 10×
- Zone classification: 5×

Система координат:
  Лидар: forward = -Y, lateral = X, up = +Z, рельс = z=0
  После rotate: forward = +X, lateral = Y
  Габарит: y ∈ [-1.35, 1.35], z ∈ [0.20, 3.40]
"""
import csv
import sys
import time
from collections import Counter
# Pool не нужен — CUDA не работает через fork
# from multiprocessing import Pool

import numpy as np
import cupy as cp
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

# ==== КОНСТАНТЫ ====
W_GAB = 1.35
GZ_BOT = 0.20
GZ_TOP = 3.40
RAIL_Z = 0.0
Z0 = RAIL_Z + GZ_BOT
Z1 = RAIL_Z + GZ_TOP

FWD_MIN = 0.3
FWD_MAX = 230.0
LAT_LIM = 3.0
NEAR_MAX = 50.0
MID_MAX = 120.0
OVH_Z = GZ_TOP

ZONE_INSIDE = "INSIDE"
ZONE_NEAR = "NEAR"
ZONE_OUTSIDE = "OUTSIDE"
ZONE_ABOVE = "ABOVE"
ZONE_RAIL = "RAIL"
ZONE_BELOW = "BELOW"
CRITICAL_ZONES = {ZONE_INSIDE, ZONE_NEAR, ZONE_RAIL, ZONE_ABOVE}


def rotate_axes_gpu(pts_gpu):
    """Rotate on GPU: forward = -Y → +X."""
    x_old = pts_gpu[:, 0].copy()
    y_old = pts_gpu[:, 1].copy()
    pts_gpu[:, 0] = -y_old
    pts_gpu[:, 1] = x_old
    return pts_gpu


def voxel_gpu(pts_gpu, vox):
    """Voxel downsample on GPU через упаковку ключей в int64.
    
    Упаковка: 21 бит на ось (максимум 2M вокселей по каждой оси).
    Для 230м / 5см = 4600 вокселей по forward — влезает.
    """
    if pts_gpu.shape[0] == 0:
        return pts_gpu
    keys = cp.floor(pts_gpu / vox).astype(cp.int64)
    # Сдвигаем чтобы все были >= 0
    keys = keys - keys.min(axis=0)
    kx = keys[:, 0]
    ky = keys[:, 1]
    kz = keys[:, 2]
    # Упаковка (kx, ky, kz) в одно int64
    packed = (kx << 42) | (ky << 21) | kz
    # Уникальные значения, индексы первого вхождения
    _, idx = cp.unique(packed, return_index=True)
    idx = cp.sort(idx)
    return pts_gpu[idx]


def preprocess_gpu(pts_gpu):
    """ROI-фильтр + voxel по слоям на GPU.
    Возвращает 3 массива numpy: near, mid, far."""
    pts_gpu = rotate_axes_gpu(pts_gpu)

    # ROI фильтр
    m = (pts_gpu[:, 0] >= FWD_MIN) & (pts_gpu[:, 0] <= FWD_MAX)
    m &= cp.abs(pts_gpu[:, 1]) <= LAT_LIM
    m &= pts_gpu[:, 2] >= -0.3
    m &= pts_gpu[:, 2] <= Z1 + 1.5
    roi = pts_gpu[m]

    if roi.shape[0] == 0:
        return (np.empty((0, 3), dtype=np.float32),
                np.empty((0, 3), dtype=np.float32),
                np.empty((0, 3), dtype=np.float32))

    # Разбивка на слои
    x = roi[:, 0]
    near = roi[(x >= FWD_MIN) & (x < NEAR_MAX)]
    mid = roi[(x >= NEAR_MAX) & (x < MID_MAX)]
    far = roi[(x >= MID_MAX) & (x <= FWD_MAX)]

    # Voxel каждый слой
    near_v = voxel_gpu(near, 0.04)
    mid_v = voxel_gpu(mid, 0.15)
    far_v = voxel_gpu(far, 0.20)

    return (cp.asnumpy(near_v).astype(np.float32),
            cp.asnumpy(mid_v).astype(np.float32),
            cp.asnumpy(far_v).astype(np.float32))


def classify_cpu(cy, cz):
    ay = abs(cy)
    if abs(cz - RAIL_Z) < 0.30:
        return ZONE_RAIL
    if cz < RAIL_Z - 0.10:
        return ZONE_BELOW
    if cz > Z1:
        return ZONE_ABOVE
    if ay <= W_GAB and Z0 <= cz <= Z1:
        return ZONE_INSIDE
    dy = ay - W_GAB
    dz_top = cz - Z1
    dz_bot = Z0 - cz
    if (0 < dy <= 0.30) or (0 < dz_top <= 0.30) or (0 < dz_bot <= 0.30):
        return ZONE_NEAR
    return ZONE_OUTSIDE


def fast_dbscan(pts, eps, min_pts):
    n = pts.shape[0]
    if n < min_pts:
        return np.full(n, -1, dtype=np.int32)
    tree = cKDTree(pts)
    pairs = tree.query_pairs(eps, output_type='ndarray')
    if len(pairs) == 0:
        return np.full(n, -1, dtype=np.int32)
    g = coo_matrix(
        (np.ones(len(pairs), dtype=np.int8), (pairs[:, 0], pairs[:, 1])),
        shape=(n, n))
    _, labels = connected_components(g, directed=False)
    counts = np.bincount(labels)
    small = counts < min_pts
    labels[small[labels]] = -1
    valid = labels >= 0
    if valid.any():
        uniq = np.unique(labels[valid])
        remap = {int(o): int(nn) for nn, o in enumerate(uniq)}
        return np.array([remap.get(int(l), -1) for l in labels], dtype=np.int32)
    return np.full(n, -1, dtype=np.int32)


def cluster_layer(pts, layer):
    """CPU DBSCAN на уже downsampled данных (быстро)."""
    if pts.shape[0] == 0:
        return []
    if layer == "near":
        eps, minp = 0.50, 3
    elif layer == "mid":
        eps, minp = 1.20, 3
    else:
        eps, minp = 4.0, 2

    if pts.shape[0] < minp:
        return []
    labels = fast_dbscan(pts, eps, minp)
    result = []
    for lbl in np.unique(labels):
        if lbl < 0:
            continue
        c = pts[labels == lbl]
        if c.shape[0] >= minp:
            result.append(c)
    return result


def is_valid_object(mn, mx, zone, dist, npts):
    size = mx - mn
    dx, dy, dz = float(size[0]), float(size[1]), float(size[2])
    maxd = max(dx, dy, dz)
    mind = max(min(dx, dy, dz), 1e-6)
    vol = dx * dy * dz
    aspect = maxd / mind
    cz_center = float((mn[2] + mx[2]) / 2)
    cy_center = float((mn[1] + mx[1]) / 2)

    if maxd > 15.0:
        return False, "too_big"
    if dist < 80.0 and vol < 0.0003:
        return False, "too_small"
    if zone == ZONE_BELOW:
        return False, "below_rail"

    if dist < 5.0:
        min_npts = 3
    elif dist < 20.0:
        min_npts = 4
    elif dist < 80.0:
        min_npts = 5
    elif dist < 120.0:
        min_npts = 4
    elif dist < 170.0:
        min_npts = 3
    else:
        min_npts = 2
    if npts < min_npts:
        return False, "few_points"

    if dist < 60.0:
        is_cube = aspect < 6.0
        is_long_flat = (maxd > 0.8) and (mind < 0.15 * maxd)
        if not (is_cube or is_long_flat):
            return False, "shape"

    if zone == ZONE_INSIDE:
        max_dy = 3.0 if dist < 80.0 else 6.0
        if dy > max_dy:
            return False, "inside_wide"
        if dz > 4.0:
            return False, "inside_tall"
        return True, "ok"
    if zone == ZONE_RAIL:
        if cz_center > 0.30:
            return False, "rail_too_high"
        if dy > 1.5:
            return False, "rail_wide"
        return True, "ok"
    if zone == ZONE_NEAR:
        if dy > 1.5:
            return False, "near_wide"
        return True, "ok"
    if zone == ZONE_ABOVE:
        if float(mn[2]) > Z1 + 0.30:
            return False, "above_too_high"
        if dy > 2.0:
            return False, "above_wide"
        if dz < 0.30:
            return False, "above_thin"
        return True, "ok"
    if zone == ZONE_OUTSIDE:
        ay = abs(cy_center)
        distance_from_gab = ay - W_GAB
        if dy > 3.0 or dz > 3.0:
            return False, "outside_wall"
        max_far = 0.5 if dist < 80.0 else 1.5
        if distance_from_gab > max_far:
            return False, "outside_far"
        return True, "ok"
    return False, "unknown"


def _process_frame_impl(ts, pts):
    t0 = time.time()
    # Переносим на GPU (один раз)
    pts_gpu = cp.asarray(pts)

    # GPU preprocess: rotate + ROI + voxel
    near, mid, far = preprocess_gpu(pts_gpu)

    # Считаем roi_points
    roi_n = near.shape[0] + mid.shape[0] + far.shape[0]

    if roi_n == 0:
        return {"ts": ts, "in_n": pts.shape[0], "roi_n": 0,
                "objects": [], "proc_ms": 0, "reject_stats": {}}

    objects = []
    reject_stats = Counter()

    # Обработка каждого слоя
    for layer, chunk in (("near", near), ("mid", mid), ("far", far)):
        for c in cluster_layer(chunk, layer):
            mn = c.min(0)
            mx = c.max(0)
            cy = float((mn[1] + mx[1]) / 2)
            cz = float((mn[2] + mx[2]) / 2)
            dist = float(max(0.0, (mn[0] + mx[0]) / 2))
            zone = classify_cpu(cy, cz)
            ok, why = is_valid_object(mn, mx, zone, dist, c.shape[0])
            if not ok:
                reject_stats[why] += 1
                continue
            center = (mn + mx) / 2.0
            size = mx - mn
            objects.append({
                "dist": float(max(0.0, center[0])),
                "cx": float(center[0]), "cy": cy, "cz": cz,
                "sx": float(size[0]), "sy": float(size[1]), "sz": float(size[2]),
                "zone": zone, "layer": layer,
                "npts": int(c.shape[0]),
            })

    # Освобождаем GPU память только периодически (free_all — медленный)
    del pts_gpu
    if not hasattr(_process_frame_impl, "_counter"):
        _process_frame_impl._counter = 0
    _process_frame_impl._counter += 1
    if _process_frame_impl._counter % 100 == 0:
        cp.get_default_memory_pool().free_all_blocks()

    elapsed_ms = (time.time() - t0) * 1000
    return {"ts": ts, "in_n": pts.shape[0], "roi_n": roi_n,
            "objects": objects, "proc_ms": elapsed_ms,
            "reject_stats": dict(reject_stats)}


def process_frame(args):
    ts, pts = args
    try:
        return _process_frame_impl(ts, pts)
    except Exception as e:
        import traceback
        return {"ts": ts, "error": str(e) + "\n" + traceback.format_exc(),
                "objects": [], "reject_stats": {}}


class Tracker:
    def __init__(self, assoc_dist=6.0, timeout=8, min_hits=15):
        self.tracks = {}
        self.next_id = 1
        self.assoc = assoc_dist
        self.timeout = timeout
        self.min_hits = min_hits

    def update(self, detections, ts):
        used = set()
        out = []
        for d in detections:
            c = np.array([d["cx"], d["cy"], d["cz"]])
            best = None
            bd = self.assoc
            for tid, tr in self.tracks.items():
                if tid in used:
                    continue
                dist = float(np.linalg.norm(c - tr["pos"]))
                if dist < bd:
                    best = tid
                    bd = dist
            if best is None:
                best = self.next_id
                self.next_id += 1
                self.tracks[best] = {"pos": c, "hits": 0, "misses": 0}
            tr = self.tracks[best]
            tr["pos"] = c
            tr["hits"] += 1
            tr["misses"] = 0
            used.add(best)
            out.append({**d, "track_id": best, "hits": tr["hits"]})

        for tid in list(self.tracks.keys()):
            if tid not in used:
                self.tracks[tid]["misses"] += 1
                if self.tracks[tid]["misses"] > self.timeout:
                    del self.tracks[tid]

        return [d for d in out if d["hits"] >= self.min_hits]


def stream_frames(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    cloud_topic = next((n for n, t in topics.items()
                        if t == "sensor_msgs/msg/PointCloud2"), None)
    if cloud_topic is None:
        raise RuntimeError("no PointCloud2")

    while reader.has_next():
        topic, data, ts = reader.read_next()
        if topic != cloud_topic:
            continue
        msg = deserialize_message(data, PointCloud2)
        if msg.width == 0 or msg.height == 0:
            continue
        n = msg.width * msg.height
        ps = msg.point_step
        buf = np.frombuffer(msg.data, dtype=np.uint8, count=n * ps)
        if len(buf) < n * ps:
            continue
        xyz = buf.reshape(n, ps)[:, :12].view(
            np.float32).reshape(n, 3).copy()
        finite = np.isfinite(xyz).all(axis=1)
        if not finite.all():
            xyz = xyz[finite]
        yield ts, xyz


def main(bag_dir, out_csv, n_workers=4):
    print(f"starting GPU pipeline on {bag_dir}...", flush=True)
    print(f"cupy {cp.__version__}, device: "
          f"{cp.cuda.runtime.getDeviceProperties(0)['name'].decode()}",
          flush=True)
    t0 = time.time()
    tracker = Tracker(assoc_dist=6.0, timeout=8, min_hits=15)
    all_tracks = {}
    total_reject = Counter()
    total_raw = 0
    processed = 0

    f = open(out_csv, "w", newline="")
    w = csv.writer(f)
    w.writerow(["frame", "ts_ns", "in_points", "roi_points", "raw_candidates",
                "confirmed", "max_dist", "dist_list", "zone_list",
                "size_list", "track_id_list", "proc_ms"])

    if True:  # single-process, GPU сам быстрый
        for res in (process_frame(f) for f in stream_frames(bag_dir)):
            processed += 1
            if res.get("error"):
                print(f"ERROR: {res['error'][:500]}", flush=True)
                continue
            dets = res["objects"]
            total_raw += len(dets)
            for k, v in res.get("reject_stats", {}).items():
                total_reject[k] += v

            confirmed = tracker.update(dets, res["ts"])
            for d in confirmed:
                tid = d["track_id"]
                if tid not in all_tracks or d["hits"] > all_tracks[tid]["hits"]:
                    all_tracks[tid] = d

            dists = sorted(round(d["dist"], 1) for d in confirmed)
            zones = [d["zone"] for d in confirmed]
            sizes = [[round(d["sx"], 2), round(d["sy"], 2), round(d["sz"], 2)]
                     for d in confirmed]
            tids = [d["track_id"] for d in confirmed]
            max_d = max(dists) if dists else 0.0

            w.writerow([processed, res["ts"], res["in_n"], res["roi_n"],
                        len(dets), len(confirmed), round(max_d, 1),
                        dists, zones, sizes, tids,
                        round(res.get("proc_ms", 0), 1)])

            if processed % 50 == 0:
                fps = processed / (time.time() - t0)
                print(f"  [{processed}] raw={len(dets)} conf={len(confirmed)} "
                      f"fps={fps:.1f} tracks={len(all_tracks)}", flush=True)

    f.close()
    dt = time.time() - t0
    print(f"\nprocessing done: {processed} frames in {dt:.1f}s "
          f"({processed/dt:.1f} fps)", flush=True)
    print(f"raw candidates: {total_raw}", flush=True)
    print(f"rejects: {dict(total_reject)}", flush=True)

    # Итоговый отчёт
    print(f"\n{'='*72}")
    print(f"  GPU PIPELINE РЕЗУЛЬТАТЫ")
    print(f"{'='*72}")

    sorted_tracks = sorted(all_tracks.values(),
                           key=lambda t: (-t["hits"], t["dist"]))
    for i, t in enumerate(sorted_tracks[:30]):
        print(f"  #{i+1:2d} dist={t['dist']:6.1f}м hits={t['hits']:3d} "
              f"zone={t['zone']:8s} size=({t['sx']:5.2f},{t['sy']:5.2f},{t['sz']:5.2f})")

    zone_cnt = Counter(t["zone"] for t in all_tracks.values())
    max_d = max((t["dist"] for t in all_tracks.values()), default=0)
    print(f"\n  Всего треков: {len(all_tracks)}")
    print(f"  По зонам: {dict(zone_cnt)}")
    print(f"  Max dist: {max_d:.1f} м")
    print(f"  Время: {dt:.1f}s ({processed/dt:.1f} fps)")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("usage: offline_pipeline_gpu.py BAG_DIR OUT_CSV [N_WORKERS]")
        sys.exit(1)
    main(sys.argv[1], sys.argv[2],
         int(sys.argv[3]) if len(sys.argv) > 3 else 4)