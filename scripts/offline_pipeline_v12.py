#!/usr/bin/env python3
"""Offline pipeline v6 — быстрый + потоковый + строгие фильтры.

Изменения от v5:
- NP.frombuffer вместо read_points → 20× быстрее загрузка
- imap_unordered → потоковая обработка, не держит 5.5 ГБ в RAM
- BELOW отсеивается полностью
- OUTSIDE: только если близко к габариту (dy<0.5)
- ABOVE: только если size_z>0.5 или свисает
- Shape-фильтр: куб или длинная тонкая плита
- min_hits=15
- Прогресс каждые 50 кадров
"""
import csv
import sys
import time
from collections import Counter
from queue import Queue
from threading import Thread
import numba

# from multiprocessing import Pool  # numba JIT не совместим с pickle

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from numba import njit

numba.config.THREADING_LAYER = 'workqueue'

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


def rotate_axes(pts):
    x_old = pts[:, 0].copy()
    y_old = pts[:, 1].copy()
    pts[:, 0] = -y_old
    pts[:, 1] = x_old
    return pts


def classify(cy, cz):
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


# ==== FAST DBSCAN ====
try:
    import open3d as o3d
except ImportError:
    o3d = None

@njit(cache=True, fastmath=True, nogil=True)
def grid_dbscan_numba(pts, eps, min_pts):
    """DBSCAN через cell hashing + union-find. O(n log n).
    Полностью numba, отпускает GIL. 5-10× быстрее open3d на наших данных."""
    n = pts.shape[0]
    if n < min_pts:
        return np.full(n, -1, dtype=np.int32)

    inv_eps = 1.0 / eps
    cells = np.empty((n, 3), dtype=np.int64)
    keys = np.empty(n, dtype=np.int64)
    for i in range(n):
        cx = np.int64(np.floor(pts[i, 0] * inv_eps))
        cy = np.int64(np.floor(pts[i, 1] * inv_eps))
        cz = np.int64(np.floor(pts[i, 2] * inv_eps))
        cells[i, 0] = cx
        cells[i, 1] = cy
        cells[i, 2] = cz
        keys[i] = ((cx + 1048576) << 42) | ((cy + 1048576) << 21) | (cz + 1048576)

    order = np.argsort(keys)
    sorted_keys = keys[order]
    sorted_cells = cells[order]
    sorted_pts = pts[order]

    parent = np.arange(n, dtype=np.int32)
    eps_sq = eps * eps

    for j in range(n):
        cj0 = sorted_cells[j, 0]
        cj1 = sorted_cells[j, 1]
        cj2 = sorted_cells[j, 2]
        px = sorted_pts[j, 0]
        py = sorted_pts[j, 1]
        pz = sorted_pts[j, 2]

        for ddx in range(-1, 2):
            cx = cj0 + ddx
            for ddy in range(-1, 2):
                cy = cj1 + ddy
                for ddz in range(-1, 2):
                    cz = cj2 + ddz
                    nk = ((cx + 1048576) << 42) | ((cy + 1048576) << 21) | (cz + 1048576)
                    lo = np.searchsorted(sorted_keys, nk, side='left')
                    hi = np.searchsorted(sorted_keys, nk, side='right')
                    for k in range(lo, hi):
                        if k <= j:
                            continue
                        dx = px - sorted_pts[k, 0]
                        dy = py - sorted_pts[k, 1]
                        dz = pz - sorted_pts[k, 2]
                        if dx * dx + dy * dy + dz * dz <= eps_sq:
                            a = j
                            while parent[a] != a:
                                parent[a] = parent[parent[a]]
                                a = parent[a]
                            b = k
                            while parent[b] != b:
                                parent[b] = parent[parent[b]]
                                b = parent[b]
                            if a != b:
                                parent[a] = b

    labels_sorted = np.empty(n, dtype=np.int32)
    for i in range(n):
        x = i
        while parent[x] != x:
            x = parent[x]
        labels_sorted[i] = x

    max_lbl = 0
    for i in range(n):
        if labels_sorted[i] > max_lbl:
            max_lbl = labels_sorted[i]
    max_lbl += 1
    counts = np.zeros(max_lbl, dtype=np.int32)
    for i in range(n):
        counts[labels_sorted[i]] += 1

    remap = np.full(max_lbl, -1, dtype=np.int32)
    next_lbl = 0
    for i in range(max_lbl):
        if counts[i] >= min_pts:
            remap[i] = next_lbl
            next_lbl += 1

    result = np.full(n, -1, dtype=np.int32)
    for i in range(n):
        result[order[i]] = remap[labels_sorted[i]]

    return result

def fast_dbscan(pts, eps, min_pts):
    """DBSCAN через open3d (C++, быстро). Fallback на scipy."""
    n = pts.shape[0]
    if n < min_pts:
        return np.full(n, -1, dtype=np.int32)

    if o3d is not None:
        # open3d работает с float64
        pc = o3d.geometry.PointCloud(
            o3d.utility.Vector3dVector(pts.astype(np.float64)))
        labels = np.asarray(pc.cluster_dbscan(
            eps=eps, min_points=min_pts, print_progress=False))
        return labels.astype(np.int32)

    # Fallback: scipy cKDTree + union-find
    from scipy.spatial import cKDTree
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components

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

@njit(cache=True, fastmath=True, nogil=True)
def voxel_hash_numba(pts, vox):
    """Voxel downsampling через хеш + argsort. 10× быстрее numpy.unique."""
    n = pts.shape[0]
    if n == 0:
        return np.empty(0, dtype=np.int64)
    inv_vox = 1.0 / vox
    keys = np.empty(n, dtype=np.int64)
    for i in range(n):
        kx = np.int64(np.floor(pts[i, 0] * inv_vox))
        ky = np.int64(np.floor(pts[i, 1] * inv_vox))
        kz = np.int64(np.floor(pts[i, 2] * inv_vox))
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

@njit(cache=True, fastmath=True, nogil=True)
def rotate_roi_numba(pts):
    """Rotate + ROI за один проход на numba. Отпускает GIL."""
    n = pts.shape[0]
    out = np.empty((n, 3), dtype=np.float32)
    k = 0
    z_max = Z1 + 1.5
    for i in range(n):
        x = -pts[i, 1]
        y = pts[i, 0]
        z = pts[i, 2]
        if x < FWD_MIN or x > FWD_MAX:
            continue
        if y < -LAT_LIM or y > LAT_LIM:
            continue
        if z < -0.30 or z > z_max:
            continue
        out[k, 0] = x
        out[k, 1] = y
        out[k, 2] = z
        k += 1
    return out[:k].copy()

@njit(cache=True, fastmath=True, nogil=True)
def extract_cluster_bboxes_numba(pts, labels, min_pts):
    """Извлекает bbox+npts каждого кластера. Заменяет np.unique + pts[labels==lbl]."""
    n = pts.shape[0]
    if n == 0:
        return np.empty((0, 6), dtype=np.float32), np.empty(0, dtype=np.int32)

    max_lbl = 0
    for i in range(n):
        if labels[i] > max_lbl:
            max_lbl = labels[i]
    max_lbl += 1

    minx = np.full(max_lbl, 1e9, dtype=np.float32)
    miny = np.full(max_lbl, 1e9, dtype=np.float32)
    minz = np.full(max_lbl, 1e9, dtype=np.float32)
    maxx = np.full(max_lbl, -1e9, dtype=np.float32)
    maxy = np.full(max_lbl, -1e9, dtype=np.float32)
    maxz = np.full(max_lbl, -1e9, dtype=np.float32)
    count = np.zeros(max_lbl, dtype=np.int32)

    for i in range(n):
        l = labels[i]
        if l < 0:
            continue
        x = pts[i, 0]; y = pts[i, 1]; z = pts[i, 2]
        if x < minx[l]: minx[l] = x
        if y < miny[l]: miny[l] = y
        if z < minz[l]: minz[l] = z
        if x > maxx[l]: maxx[l] = x
        if y > maxy[l]: maxy[l] = y
        if z > maxz[l]: maxz[l] = z
        count[l] += 1

    bboxes = np.empty((max_lbl, 6), dtype=np.float32)
    npts_arr = np.empty(max_lbl, dtype=np.int32)
    n_valid = 0
    for l in range(max_lbl):
        if count[l] >= min_pts:
            bboxes[n_valid, 0] = minx[l]
            bboxes[n_valid, 1] = miny[l]
            bboxes[n_valid, 2] = minz[l]
            bboxes[n_valid, 3] = maxx[l]
            bboxes[n_valid, 4] = maxy[l]
            bboxes[n_valid, 5] = maxz[l]
            npts_arr[n_valid] = count[l]
            n_valid += 1
    return bboxes[:n_valid].copy(), npts_arr[:n_valid].copy()

def cluster_layer(pts, layer):
    """Возвращает список (mn, mx, npts) — без копирования точек."""
    if pts.shape[0] == 0:
        return []
    if layer == "near":
        eps, minp, vox = 0.50, 3, 0.10
    elif layer == "mid":
        eps, minp, vox = 1.20, 3, 0.25
    else:
        eps, minp, vox = 4.0, 2, 0.35

    if vox > 0 and pts.shape[0] > minp:
        idx = voxel_hash_numba(pts, vox)
        pts = pts[idx]

    if pts.shape[0] < minp:
        return []

    labels = grid_dbscan_numba(pts, eps, minp)
    if labels.shape[0] == 0:
        return []

    # numba bbox extract — вместо np.unique + цикла (отпускает GIL)
    bboxes, npts_arr = extract_cluster_bboxes_numba(pts, labels, minp)

    return [(bboxes[i, 0:3], bboxes[i, 3:6], int(npts_arr[i]))
            for i in range(bboxes.shape[0])]

# ==== ФИЛЬТР v6 ====
def is_valid_object(mn, mx, zone, dist, npts):
    """Возвращает (ok, reason). Строгие правила для габаритно-рельсовой задачи."""
    size = mx - mn
    dx, dy, dz = float(size[0]), float(size[1]), float(size[2])
    maxd = max(dx, dy, dz)
    mind = max(min(dx, dy, dz), 1e-6)
    vol = dx * dy * dz
    aspect = maxd / mind

    cz_center = float((mn[2] + mx[2]) / 2)
    cy_center = float((mn[1] + mx[1]) / 2)

    # --- АБСОЛЮТНЫЕ ЛИМИТЫ ---
    if maxd > 15.0:
        return False, "too_big"
    # Для дальних — не проверяем объём (на 150 м точки могут быть разрежены)
    if dist < 80.0 and vol < 0.0003:
        return False, "too_small"

    # --- BELOW всегда отсеиваем ---
    if zone == ZONE_BELOW:
        return False, "below_rail"

        # --- ПОРОГ POINTS по дистанции ---
        # Дистанционно-адаптивный порог: чем дальше, тем меньше нужно точек
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
    else:  # 170+ м
        min_npts = 2
    if npts < min_npts:
        return False, "few_points"

    # --- SHAPE-ФИЛЬТР: только для близких, дальним доверяем ---
    if dist < 60.0:
        is_cube = aspect < 6.0
        is_long_flat = (maxd > 0.8) and (mind < 0.15 * maxd)
        if not (is_cube or is_long_flat):
            return False, "shape"

    # --- INSIDE: критично, но ограничить ---
    if zone == ZONE_INSIDE:
        # Для дальних объектов допускаем большую ширину — это может быть стена
        max_dy = 3.0 if dist < 80.0 else 6.0
        if dy > max_dy:
            return False, "inside_wide"
        if dz > 4.0:
            return False, "inside_tall"
        return True, "ok"

    # --- RAIL: низкий тонкий на рельсе ---
    if zone == ZONE_RAIL:
        if cz_center > 0.30:      # на рельсе объект не должен быть высоко
            return False, "rail_too_high"
        if dy > 1.5:
            return False, "rail_wide"
        return True, "ok"

    # --- NEAR: на границе, близко к габариту ---
    if zone == ZONE_NEAR:
        if dy > 1.5:
            return False, "near_wide"
        return True, "ok"

    # --- ABOVE: только если реально свисает в габарит ---
    if zone == ZONE_ABOVE:
        # Проверяем: спускается ли объект в габарит?
        z_bottom = float(mn[2])
        if z_bottom > Z1 + 0.30:
            return False, "above_too_high"
        if dy > 2.0:
            return False, "above_wide"
        if dz < 0.30:
            return False, "above_thin"
        return True, "ok"

    # --- OUTSIDE: только если очень близко к габариту ---
    if zone == ZONE_OUTSIDE:
        ay = abs(cy_center)
        distance_from_gab = ay - W_GAB
        if dy > 3.0 or dz > 3.0:
            return False, "outside_wall"
        # Для дальних — допускаем до 1 м от габарита (это может быть стена тоннеля)
        max_far = 0.5 if dist < 80.0 else 1.5
        if distance_from_gab > max_far:
            return False, "outside_far"
        return True, "ok"

    return False, "unknown_zone"

def _reader_thread(bag_dir, q):
    """Поток-читатель: кладёт кадры в очередь."""
    try:
        for frame in stream_frames(bag_dir):
            q.put(frame)  # блокируется если очередь полна
    except Exception as e:
        import traceback
        print(f"reader error: {e}\n{traceback.format_exc()}", flush=True)
    finally:
        q.put(None)  # sentinel — конец потока


def _frame_gen_from_queue(q):
    """Генератор из очереди для ThreadPoolExecutor.map."""
    while True:
        item = q.get()
        if item is None:
            break
        yield item



def _process_frame_impl(ts, pts):
    t0 = time.time()
    roi = rotate_roi_numba(pts)
    roi_n = roi.shape[0]
    if roi_n == 0:
        return {"ts": ts, "in_n": pts.shape[0], "roi_n": 0,
                "objects": [], "proc_ms": 0, "reject_stats": {}}

    objects = []
    reject_stats = Counter()

    ovh = roi[roi[:, 2] > OVH_Z + 0.1]
    # Пропускаем overhang-обработку, если нет точек сверху — экономит 20-30% времени
    if ovh.shape[0] < 5:
        pass
    else:
        for mn, mx, npts in cluster_layer(ovh, "near"):
            cy = float((mn[1] + mx[1]) / 2)
            cz = float((mn[2] + mx[2]) / 2)
            dist = float(max(0.0, (mn[0] + mx[0]) / 2))
            ok, why = is_valid_object(mn, mx, ZONE_ABOVE, dist, npts)
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
                "npts": int(npts),
            })

    near = roi[(roi[:, 0] >= FWD_MIN) & (roi[:, 0] < NEAR_MAX)]
    mid = roi[(roi[:, 0] >= NEAR_MAX) & (roi[:, 0] < MID_MAX)]
    far = roi[(roi[:, 0] >= MID_MAX) & (roi[:, 0] <= FWD_MAX)]

    for layer, chunk in (("near", near), ("mid", mid), ("far", far)):
        for mn, mx, npts in cluster_layer(chunk, layer):
            cy = float((mn[1] + mx[1]) / 2)
            cz = float((mn[2] + mx[2]) / 2)
            dist = float(max(0.0, (mn[0] + mx[0]) / 2))
            zone = classify(cy, cz)
            ok, why = is_valid_object(mn, mx, zone, dist, npts)
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
                "npts": int(npts),
            })

    return {"ts": ts, "in_n": pts.shape[0], "roi_n": roi_n,
            "objects": objects, "proc_ms": (time.time() - t0) * 1000,
            "reject_stats": dict(reject_stats)}


def process_frame(args):
    ts, pts = args
    try:
        return _process_frame_impl(ts, pts)
    except Exception as e:
        return {"ts": ts, "error": str(e), "objects": [], "reject_stats": {}}


def _make_obj(c, zone, layer, mn, mx, cy, cz):
    center = (mn + mx) / 2.0
    size = mx - mn
    vol = float(size[0] * size[1] * size[2]) + 1e-9
    return {
        "dist": float(max(0.0, center[0])),
        "cx": float(center[0]), "cy": cy, "cz": cz,
        "sx": float(size[0]), "sy": float(size[1]), "sz": float(size[2]),
        "zone": zone, "layer": layer,
        "npts": int(c.shape[0]),
    }


# ==== TRACKER ====
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


# ==== БЫСТРАЯ ЗАГРУЗКА ====
def stream_frames(bag_dir):
    """Генератор (ts, pts). pts: Nx3 float32 (только x,y,z).
    Извлекает x,y,z из PointCloud2.data через numpy — 20× быстрее read_points.
    """
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

    frame_idx = 0
    while reader.has_next():
        topic, data, ts = reader.read_next()
        if topic != cloud_topic:
            continue
        msg = deserialize_message(data, PointCloud2)
        
        # Быстрое извлечение xyz: data → uint8 [N, point_step] → первые 12 байт → 3×float32
        n = msg.width * msg.height
        ps = msg.point_step
        buf = np.frombuffer(msg.data, dtype=np.uint8, count=n * ps)
        if len(buf) < n * ps:
            continue
        xyz = buf.reshape(n, ps)[:, :12].view(np.float32).reshape(n, 3).copy()
        # intensity filter отключён: см. анализ в docs/BENCHMARK.md
        finite = np.isfinite(xyz).all(axis=1)
        if not finite.all():
            xyz = xyz[finite]
        yield ts, xyz


def stream_loader(bag_dir, count):
    """Возвращает generator для Pool.imap с индексацией."""
    for i, (ts, pts) in enumerate(stream_frames(bag_dir)):
        yield ts, pts


def main(bag_dir, out_csv, n_workers=4):
        # Пропускаем подсчёт — обрабатываем сразу в потоке
    print(f"starting streaming processing of {bag_dir}...", flush=True)
    total_frames = 0
    t0 = time.time()

    # === Потоковая обработка ===
    print(f"processing (workers={n_workers}, streaming)...", flush=True)
    t0 = time.time()
    tracker = Tracker(assoc_dist=6.0, timeout=8, min_hits=15)
    all_tracks = {}
    total_reject = Counter()
    total_raw = 0

    f = open(out_csv, "w", newline="")
    w = csv.writer(f)
    w.writerow(["frame", "ts_ns", "in_points", "roi_points", "raw_candidates",
                "confirmed", "max_dist", "dist_list", "zone_list",
                "size_list", "track_id_list", "proc_ms"])

    processed = 0
    
    # Прогрев numba в главном процессе — чтобы воркеры получили закешированную функцию
        # Прогрев numba в главном процессе
    _dummy = np.random.randn(2000, 3).astype(np.float32)
    _ = voxel_hash_numba(_dummy, 0.08)
    _ = rotate_roi_numba(_dummy)
    _ = grid_dbscan_numba(_dummy[:500], 0.5, 3)
    print("  numba warmed", flush=True)

    # === PREFETCH I/O ===
    # Отдельный поток читает bag, воркеры обрабатывают из очереди
    q = Queue(maxsize=32)
    reader = Thread(target=_reader_thread, args=(bag_dir, q), daemon=True)
    reader.start()
    print("  reader started", flush=True)

    from concurrent.futures import ThreadPoolExecutor
    with ThreadPoolExecutor(max_workers=n_workers) as executor:
        for res in executor.map(process_frame, _frame_gen_from_queue(q)):
            processed += 1
            if res.get("error"):
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
                print(f"  [{processed} frames] raw={len(dets)} "
                      f"conf={len(confirmed)} fps={fps:.1f} "
                      f"tracks={len(all_tracks)}", flush=True)

    f.close()
    dt = time.time() - t0
    print(f"\nprocessing done: {processed} frames in {dt:.1f}s "
          f"({processed/dt:.1f} fps)", flush=True)
    print(f"raw candidates: {total_raw}", flush=True)
    print(f"rejects: {dict(total_reject)}", flush=True)

        # === ПОСТ-ФИЛЬТР: слияние треков по позиции ===
    print(f"\n{'='*72}")
    print(f"  ФИЛЬТРАЦИЯ ТРЕКОВ")
    print(f"{'='*72}")
    print(f"  До фильтрации: {len(all_tracks)} треков")

    # Фильтр 1: минимальный hits
        # Для дальних объектов hits может быть небольшим — они видны меньше времени
    def min_hits_for(t):
        if t["dist"] < 50:
            return 50
        elif t["dist"] < 100:
            return 30
        elif t["dist"] < 150:
            return 15
        else:
            return 8
    filtered = [t for t in all_tracks.values()
                if t["hits"] >= min_hits_for(t)]
    print(f"  После hits-фильтра: {len(filtered)}")

    # Фильтр 2: RAIL — только объёмные (не рельсы)
    filtered = [t for t in filtered
                if t["zone"] != "RAIL" or (t["sy"] >= 0.20 or t["sz"] >= 0.20)]
    print(f"  После отсева тонких рельс: {len(filtered)}")

    # Фильтр 3: минимальный размер
    filtered = [t for t in filtered
                if max(t["sx"], t["sy"], t["sz"]) >= 0.15]
    print(f"  После размера>=0.15м: {len(filtered)}")

    # Фильтр 4: кластеризация треков в объекты
    # Объекты на разных дистанциях (по forward), кластеры по (cx, cy, cz) с eps=3м
    clusters = []
    used = set()
    # Сортируем по hits (больше = стабильнее = настоящий объект)
    filtered.sort(key=lambda t: -t["hits"])
    for t in filtered:
        if id(t) in used:
            continue
        # Найти все треки рядом с ним
        group = [t]
        used.add(id(t))
        c1 = np.array([t["cx"], t["cy"], t["cz"]])
        for t2 in filtered:
            if id(t2) in used:
                continue
            c2 = np.array([t2["cx"], t2["cy"], t2["cz"]])
            if np.linalg.norm(c1 - c2) < 3.0:
                group.append(t2)
                used.add(id(t2))
        # Суммарные hits
        total_hits = sum(g["hits"] for g in group)
        # Лучший трек = с максимум hits
        best = max(group, key=lambda g: g["hits"])
        clusters.append({
            **best,
            "total_hits": total_hits,
            "n_tracks": len(group),
        })

    print(f"  После кластеризации по позиции: {len(clusters)} уникальных объектов")

        # === РАНЖИРОВАНИЕ ОБЪЕКТОВ ===
    # Score = total_hits × log(1+npts) × n_tracks — устойчивее шума
    for c in clusters:
        c["score"] = c["total_hits"] * np.log1p(c["npts"]) * c["n_tracks"]

    clusters.sort(key=lambda c: -c["score"])

    print(f"\n{'='*72}")
    print(f"  TOP-15 ОБЪЕКТОВ (по score = hits × log(npts) × group)")
    print(f"{'='*72}")
    for i, t in enumerate(clusters[:15]):
        print(f"  #{i+1:2d}  dist={t['dist']:6.1f}м  score={t['score']:8.0f}  "
              f"hits={t['total_hits']:4d}  group={t['n_tracks']:2d}  "
              f"zone={t['zone']:8s}  size=({t['sx']:5.2f},{t['sy']:5.2f},{t['sz']:5.2f})  "
              f"npts={t['npts']}")

    # Распределение по дистанции (по bins)
    print(f"\n{'='*72}")
    print(f"  РАСПРЕДЕЛЕНИЕ ПО ДИСТАНЦИИ")
    print(f"{'='*72}")
    bins = [(0, 20), (20, 50), (50, 100), (100, 150), (150, 230)]
    for lo, hi in bins:
        n_bin = sum(1 for t in clusters if lo <= t["dist"] < hi)
                # Для дальних порог ниже, т.к. объект виден меньше кадров
        if lo < 100:
            min_real_hits = 300
        else:
            min_real_hits = 60   # на 100+ м объект мелькает реже
        n_real = sum(1 for t in clusters
                     if lo <= t["dist"] < hi and t["total_hits"] > min_real_hits)
        bar = "#" * min(n_bin, 40)
        print(f"  {lo:3d}-{hi:3d}м: {n_bin:3d} (real: {n_real}) {bar}")

    zone_count = Counter(t["zone"] for t in clusters)
    max_dist = max((t["dist"] for t in clusters), default=0)

    print(f"\n  Всего уникальных: {len(clusters)}")
    print(f"  Сильных (hits>300): {sum(1 for t in clusters if t['total_hits'] > 300)}")
    print(f"  По зонам: {dict(zone_count)}")
    print(f"  Max dist: {max_dist:.1f} м")
    print(f"  Обработка: {dt:.1f}s ({processed/dt:.1f} fps)")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("usage: offline_pipeline.py BAG_DIR OUT_CSV [N_WORKERS]")
        sys.exit(1)
    main(sys.argv[1], sys.argv[2],
         int(sys.argv[3]) if len(sys.argv) > 3 else 4)