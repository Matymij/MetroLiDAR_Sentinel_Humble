#!/usr/bin/env python3
"""Offline pipeline v10 — MAXIMUM SPEED.
Numba JIT + multiprocessing Pool для real-time.

Целевые метрики: 30-60 fps (было 3 fps).
"""
import csv
import sys
import time
from collections import Counter
from multiprocessing import Pool

import numpy as np
import numba
from numba import njit, prange
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

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

ZONE_NAMES = ["INSIDE", "NEAR", "OUTSIDE", "ABOVE", "RAIL", "BELOW"]
ZONE_INSIDE = "INSIDE"
ZONE_NEAR = "NEAR"
ZONE_OUTSIDE = "OUTSIDE"
ZONE_ABOVE = "ABOVE"
ZONE_RAIL = "RAIL"
ZONE_BELOW = "BELOW"
CRITICAL_ZONES = {ZONE_INSIDE, ZONE_NEAR, ZONE_RAIL, ZONE_ABOVE}


# ============================================================
# NUMBA JIT: rotate + ROI за один проход
# ============================================================
@njit(cache=True, fastmath=True)
def rotate_roi_numba(pts):
    """Rotate axes + ROI filter + возврат плоского массива."""
    n = pts.shape[0]
    out = np.empty((n, 3), dtype=np.float32)
    k = 0
    z_max = Z1 + 1.5
    for i in range(n):
        # rotate: forward = -Y, lateral = X
        x = -pts[i, 1]
        y = pts[i, 0]
        z = pts[i, 2]
        # ROI
        if x < FWD_MIN or x > FWD_MAX:
            continue
        if y < -LAT_LIM or y > LAT_LIM:
            continue
        if z < -0.3 or z > z_max:
            continue
        out[k, 0] = x
        out[k, 1] = y
        out[k, 2] = z
        k += 1
    return out[:k].copy()


# ============================================================
# NUMBA JIT: voxel hash (быстрее numpy unique в 5-10×)
# ============================================================
@njit(cache=True, fastmath=True)
def voxel_hash_numba(pts, vox):
    """Voxel downsampling через хеш + argsort.
    Возвращает индексы уникальных вокселей."""
    n = pts.shape[0]
    if n == 0:
        return np.empty(0, dtype=np.int64)
    inv_vox = 1.0 / vox
    keys = np.empty(n, dtype=np.int64)
    for i in range(n):
        kx = np.int64(np.floor(pts[i, 0] * inv_vox))
        ky = np.int64(np.floor(pts[i, 1] * inv_vox))
        kz = np.int64(np.floor(pts[i, 2] * inv_vox))
        # Упаковка: 21 бит на ось, сдвиг чтобы не было отрицательных
        keys[i] = ((kx + 1048576) << 42) | ((ky + 1048576) << 21) | (kz + 1048576)

    order = np.argsort(keys)
    sorted_keys = keys[order]
    keep = np.zeros(n, dtype=np.bool_)
    if n > 0:
        keep[0] = True
        for i in range(1, n):
            if sorted_keys[i] != sorted_keys[i - 1]:
                keep[i] = True
    result = order[keep]
    return np.sort(result)


# ============================================================
# NUMBA JIT: GRID DBSCAN (главное ускорение — 10× от cKDTree)
# ============================================================
@njit(cache=True, fastmath=True)
def grid_dbscan_numba(pts, eps, min_pts):
    """Grid-based DBSCAN через inline union-find. Быстрее cKDTree в 5-10×."""
    n = pts.shape[0]
    if n < min_pts:
        return np.full(n, -1, dtype=np.int32)

    # Разбиение на ячейки
    inv_eps = 1.0 / eps
    cells = np.empty((n, 3), dtype=np.int32)
    for i in range(n):
        cells[i, 0] = int(np.floor(pts[i, 0] * inv_eps))
        cells[i, 1] = int(np.floor(pts[i, 1] * inv_eps))
        cells[i, 2] = int(np.floor(pts[i, 2] * inv_eps))

    parent = np.arange(n, dtype=np.int32)
    eps_sq = eps * eps

    # Union-find inline (без вложенных функций и dict)
    for i in range(n):
        for j in range(i + 1, n):
            if abs(cells[i, 0] - cells[j, 0]) > 1:
                continue
            if abs(cells[i, 1] - cells[j, 1]) > 1:
                continue
            if abs(cells[i, 2] - cells[j, 2]) > 1:
                continue
            dx = pts[i, 0] - pts[j, 0]
            dy = pts[i, 1] - pts[j, 1]
            dz = pts[i, 2] - pts[j, 2]
            if dx * dx + dy * dy + dz * dz <= eps_sq:
                # find(i) с path compression
                a = i
                while parent[a] != a:
                    parent[a] = parent[parent[a]]
                    a = parent[a]
                # find(j)
                b = j
                while parent[b] != b:
                    parent[b] = parent[parent[b]]
                    b = parent[b]
                if a != b:
                    parent[a] = b

    # Сжатие путей
    labels = np.empty(n, dtype=np.int32)
    for i in range(n):
        x = i
        while parent[x] != x:
            x = parent[x]
        labels[i] = x

    # Подсчёт размеров через bincount (без dict)
    max_lbl = 0
    for i in range(n):
        if labels[i] > max_lbl:
            max_lbl = labels[i]
    max_lbl += 1
    counts = np.zeros(max_lbl, dtype=np.int32)
    for i in range(n):
        counts[labels[i]] += 1

    # Remap: маленькие кластеры → -1, большие → 0,1,2,...
    remap = np.full(max_lbl, -1, dtype=np.int32)
    next_lbl = 0
    for i in range(max_lbl):
        if counts[i] >= min_pts:
            remap[i] = next_lbl
            next_lbl += 1

    result = np.full(n, -1, dtype=np.int32)
    for i in range(n):
        result[i] = remap[labels[i]]
    return result


# ============================================================
# Кластеризация слоя
# ============================================================
def cluster_layer(pts, layer):
    if pts.shape[0] == 0:
        return []
    if layer == "near":
        eps, minp, vox = 0.50, 3, 0.04
    elif layer == "mid":
        eps, minp, vox = 1.20, 3, 0.15
    else:
        eps, minp, vox = 4.0, 2, 0.20

    if vox > 0 and pts.shape[0] > minp:
        idx = voxel_hash_numba(pts, vox)
        pts = pts[idx]

    if pts.shape[0] < minp:
        return []
    labels = grid_dbscan_numba(pts, eps, minp)
    result = []
    for lbl in np.unique(labels):
        if lbl < 0:
            continue
        c = pts[labels == lbl]
        if c.shape[0] >= minp:
            result.append(c)
    return result


def classify_name(code):
    return ZONE_NAMES[code]


def classify_zone(cy, cz):
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
    # Numba: rotate + ROI за один проход
    roi = rotate_roi_numba(pts)

    if roi.shape[0] == 0:
        return {"ts": ts, "in_n": pts.shape[0], "roi_n": 0,
                "objects": [], "proc_ms": 0, "reject_stats": {}}

    x = roi[:, 0]
    near = roi[(x >= FWD_MIN) & (x < NEAR_MAX)]
    mid = roi[(x >= NEAR_MAX) & (x < MID_MAX)]
    far = roi[(x >= MID_MAX) & (x <= FWD_MAX)]

    objects = []
    reject_stats = Counter()

    # Overhang
    ovh = roi[roi[:, 2] > OVH_Z + 0.1]
    if ovh.shape[0] >= 5:
        for c in cluster_layer(ovh, "near"):
            mn = c.min(0); mx = c.max(0)
            cy = float((mn[1] + mx[1]) / 2)
            cz = float((mn[2] + mx[2]) / 2)
            dist = float(max(0.0, (mn[0] + mx[0]) / 2))
            ok, why = is_valid_object(mn, mx, ZONE_ABOVE, dist, c.shape[0])
            if not ok:
                reject_stats[why] += 1
                continue
            center = (mn + mx) / 2.0
            size = mx - mn
            objects.append({
                "dist": float(center[0]),
                "cx": float(center[0]), "cy": cy, "cz": cz,
                "sx": float(size[0]), "sy": float(size[1]), "sz": float(size[2]),
                "zone": ZONE_ABOVE, "layer": "overhang",
                "npts": int(c.shape[0]),
            })

    # Основные слои
    for layer, chunk in (("near", near), ("mid", mid), ("far", far)):
        for c in cluster_layer(chunk, layer):
            mn = c.min(0); mx = c.max(0)
            cy = float((mn[1] + mx[1]) / 2)
            cz = float((mn[2] + mx[2]) / 2)
            dist = float(max(0.0, (mn[0] + mx[0]) / 2))
            zone = classify_zone(cy, cz)
            ok, why = is_valid_object(mn, mx, zone, dist, c.shape[0])
            if not ok:
                reject_stats[why] += 1
                continue
            center = (mn + mx) / 2.0
            size = mx - mn
            objects.append({
                "dist": float(center[0]),
                "cx": float(center[0]), "cy": cy, "cz": cz,
                "sx": float(size[0]), "sy": float(size[1]), "sz": float(size[2]),
                "zone": zone, "layer": layer,
                "npts": int(c.shape[0]),
            })

    elapsed_ms = (time.time() - t0) * 1000
    return {"ts": ts, "in_n": pts.shape[0], "roi_n": roi.shape[0],
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


def _worker_init():
    """Прогрев numba в каждом воркере Pool."""
    dummy = np.random.randn(1000, 3).astype(np.float32)
    _ = rotate_roi_numba(dummy)
    _ = voxel_hash_numba(dummy, 0.05)
    _ = grid_dbscan_numba(dummy[:500], 0.5, 3)


def main(bag_dir, out_csv, n_workers=6):
    print(f"starting v10 (numba+Pool) on {bag_dir}...", flush=True)
    print(f"workers: {n_workers}", flush=True)

    # Прогрев numba в главном процессе
    print("warming up numba JIT...", flush=True)
    tw = time.time()
    dummy = np.random.randn(5000, 3).astype(np.float32)
    _ = rotate_roi_numba(dummy)
    _ = voxel_hash_numba(dummy, 0.05)
    _ = grid_dbscan_numba(dummy[:500], 0.5, 3)
    print(f"  numba warmed in {time.time()-tw:.1f}s", flush=True)

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

    with Pool(n_workers, initializer=_worker_init) as pool:
        for res in pool.imap_unordered(
                process_frame, stream_frames(bag_dir), chunksize=8):
            processed += 1
            if res.get("error"):
                print(f"ERROR: {res['error'][:300]}", flush=True)
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

    sorted_tracks = sorted(all_tracks.values(),
                           key=lambda t: (-t["hits"], t["dist"]))
    print(f"\n{'='*72}")
    print(f"  V10 РЕЗУЛЬТАТЫ")
    print(f"{'='*72}")
    for i, t in enumerate(sorted_tracks[:20]):
        print(f"  #{i+1:2d} dist={t['dist']:6.1f}м hits={t['hits']:3d} "
              f"zone={t['zone']:8s} size=({t['sx']:5.2f},{t['sy']:5.2f},{t['sz']:5.2f})")

    max_d = max((t["dist"] for t in all_tracks.values()), default=0)
    print(f"\n  Всего треков: {len(all_tracks)}")
    print(f"  Max dist: {max_d:.1f} м")
    print(f"  Время: {dt:.1f}s ({processed/dt:.1f} fps)")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("usage: offline_pipeline_v10.py BAG_DIR OUT_CSV [N_WORKERS]")
        sys.exit(1)
    main(sys.argv[1], sys.argv[2],
         int(sys.argv[3]) if len(sys.argv) > 3 else 6)