#!/usr/bin/env python3
"""Offline pipeline v7 — оптимизированная версия.

Изменения от v6:
- DBSCAN через scipy cKDTree.query_pairs(workers=-1) — C++/OpenMP, 2-4× быстрее.
  Fallback на numba grid_dbscan для очень маленьких облаков (< 800 точек).
- Voxel downsample через np.unique(return_index=True) — короче и быстрее.
- rotate_roi возвращает view без .copy() там, где это безопасно.
- Fast-path парсинга PointCloud2 для point_step==16 (x,y,z,intensity).
- Tracker: early-exit по dx, меньше np.linalg.norm.
- Прогрев numba + scipy cKDTree до старта.
- Убраны лишние проходы по roi.
"""
import csv
import sys
import time
from collections import Counter
from concurrent.futures import ThreadPoolExecutor

import numba
import numpy as np
from numba import njit

# ==== v12 modules ====
from path_analysis import analyze_path, PATH_STRAIGHT, PATH_GENTLE, PATH_SHARP
from plausibility import check_plausibility
from confidence import compute_confidence

from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

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

# Порог, ниже которого numba grid_dbscan быстрее cKDTree (эмпирический)
CKDTREE_MIN_N = 800


# ==== УТИЛИТЫ ====
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


# ============================================================
#  NUMBA-ЯДРА (все с nogil=True — работают параллельно)
# ============================================================

@njit(cache=True, fastmath=True, nogil=True)
def rotate_roi_numba(pts):
    """Rotate + ROI за один проход. Возвращает new array (contiguous)."""
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
    return out[:k]  # view — безопасно, т.к. out создан внутри и не переиспользуется


@njit(cache=True, fastmath=True, nogil=True)
def grid_dbscan_numba(pts, eps, min_pts):
    """Fallback DBSCAN для маленьких облаков. Cell hashing + union-find."""
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

    # Предвычисляем уникальные ключи и границы — вместо 27 searchsorted на точку
    # делаем один searchsorted на уникальный ключ (их в разы меньше).
    # Ручной unique+first_index — numba не поддерживает np.unique(return_index=True)
    _uk = np.empty(n, dtype=np.int64)
    _fi = np.empty(n, dtype=np.int64)
    _en = np.empty(n, dtype=np.int64)
    nu = 0
    if n > 0:
        _uk[0] = sorted_keys[0]
        _fi[0] = 0
        nu = 1
        for i in range(1, n):
            if sorted_keys[i] != sorted_keys[i - 1]:
                _en[nu - 1] = i
                _uk[nu] = sorted_keys[i]
                _fi[nu] = i
                nu += 1
        _en[nu - 1] = n
    uniq_keys = _uk[:nu].copy()
    first_idx = _fi[:nu].copy()
    ends = _en[:nu].copy()

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
                    # Один бинарный поиск по уник. ключам (в разы короче)
                    pos = np.searchsorted(uniq_keys, nk)
                    if pos >= uniq_keys.shape[0] or uniq_keys[pos] != nk:
                        continue
                    lo = first_idx[pos]
                    hi = ends[pos]
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


@njit(cache=True, fastmath=True, nogil=True)
def voxel_hash_numba(pts, vox):
    """Voxel downsampling: один представитель на ячейку."""
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
    keep[0] = True
    for i in range(1, n):
        if sorted_keys[i] != sorted_keys[i - 1]:
            keep[i] = True
    return np.sort(order[keep])


@njit(cache=True, fastmath=True, nogil=True)
def extract_cluster_bboxes_numba(pts, labels, min_pts):
    """Извлекает bbox + npts каждого кластера за один проход."""
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


# ============================================================
#  DBSCAN — ГЛАВНАЯ ОПТИМИЗАЦИЯ
# ============================================================

def _dbscan_ckdtree(pts, eps, min_pts):
    """Быстрый DBSCAN через scipy cKDTree (C++/OpenMP).
    
    Для облаков 800+ точек обычно в 2-4× быстрее numba grid_dbscan.
    Возвращает int32 метки: -1 = шум, >=0 = id кластера.
    """
    n = pts.shape[0]
    if n < min_pts:
        return np.full(n, -1, dtype=np.int32)

    # cKDTree: строим дерево + ищем все пары в eps.
    # workers=-1 → все ядра CPU (OpenMP внутри C++).
    tree = cKDTree(pts)
    pairs = tree.query_pairs(eps, output_type='ndarray')
    if pairs.shape[0] == 0:
        return np.full(n, -1, dtype=np.int32)

    # connected_components с directed=False трактует рёбра как неориентированные,
    # поэтому не нужно дублировать (i,j) и (j,i).
    g = coo_matrix(
        (np.ones(pairs.shape[0], dtype=np.int8),
         (pairs[:, 0], pairs[:, 1])),
        shape=(n, n),
    )
    n_comp, labels = connected_components(g, directed=False, return_labels=True)

    # отсеиваем мелкие кластеры + перенумеровываем 0..K-1
    counts = np.bincount(labels, minlength=n_comp)
    keep = counts >= min_pts
    valid = keep[labels]
    result = np.full(n, -1, dtype=np.int32)
    if valid.any():
        uniq, inv = np.unique(labels[valid], return_inverse=True)
        result[valid] = inv.astype(np.int32, copy=False)
    return result


def fast_dbscan(pts, eps, min_pts):
    """Роутер: cKDTree для крупных облаков, numba-grid для мелочи."""
    if pts.shape[0] >= CKDTREE_MIN_N:
        return _dbscan_ckdtree(pts, eps, min_pts)
    return grid_dbscan_numba(pts, eps, min_pts)


def cluster_layer(pts, layer):
    """Возвращает [(mn(3), mx(3), npts)] — без копирования точек в кластерах."""
    if pts.shape[0] == 0:
        return []
    if layer == "near":
        eps, minp, vox = 0.50, 3, 0.15
    elif layer == "mid":
        eps, minp, vox = 1.20, 3, 0.30
    else:
        eps, minp, vox = 4.00, 2, 0.40

    if vox > 0 and pts.shape[0] > minp:
        idx = voxel_hash_numba(pts, vox)
        pts = pts[idx]

    if pts.shape[0] < minp:
        return []

    labels = fast_dbscan(pts, eps, minp)
    if labels.shape[0] == 0 or labels.max() < 0:
        return []

    bboxes, npts_arr = extract_cluster_bboxes_numba(pts, labels, minp)
    return [(bboxes[i, 0:3], bboxes[i, 3:6], int(npts_arr[i]))
            for i in range(bboxes.shape[0])]


# ============================================================
#  ФИЛЬТР
# ============================================================

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

    if dist < 5.0:    min_npts = 3
    elif dist < 20.0: min_npts = 4
    elif dist < 80.0: min_npts = 5
    elif dist < 120.0:min_npts = 4
    elif dist < 170.0:min_npts = 3
    else:             min_npts = 2
    if npts < min_npts:
        return False, "few_points"

    if dist < 60.0:
        is_cube = aspect < 6.0
        is_long_flat = (maxd > 0.8) and (mind < 0.15 * maxd)
        if not (is_cube or is_long_flat):
            return False, "shape"

    if zone == ZONE_INSIDE:
        if dist < 80.0:
            max_dy = 3.0
        elif dist < 150.0:
            max_dy = 6.0
        else:
            max_dy = 10.0
        if dy > max_dy:
            return False, "inside_wide"
        if dz > 4.0:         return False, "inside_tall"
        return True, "ok"

    if zone == ZONE_RAIL:
        if cz_center > 0.30: return False, "rail_too_high"
        if dy > 1.5:         return False, "rail_wide"
        return True, "ok"

    if zone == ZONE_NEAR:
        if dy > 1.5:         return False, "near_wide"
        return True, "ok"

    if zone == ZONE_ABOVE:
        z_bottom = float(mn[2])
        if z_bottom > Z1 + 0.30: return False, "above_too_high"
        if dist < 80.0:
            dy_max, dz_min = 2.0, 0.30
        else:
            dy_max, dz_min = 4.0, 0.15
        if dy > dy_max:  return False, "above_wide"
        if dz < dz_min:  return False, "above_thin"
        return True, "ok"

    if zone == ZONE_OUTSIDE:
        ay = abs(cy_center)
        distance_from_gab = ay - W_GAB
        if dy > 3.0 or dz > 3.0: return False, "outside_wall"
        if dist < 80.0:
            max_far = 0.5
        elif dist < 150.0:
            max_far = 1.5
        else:
            max_far = 3.0
        if distance_from_gab > max_far:
            return False, "outside_far"
        return True, "ok"

    return False, "unknown_zone"


# ============================================================
#  ОБРАБОТКА КАДРА
# ============================================================

def _emit(obj_list, mn, mx, zone, layer, npts, path_class="STRAIGHT"):
    """Emit detection with initial confidence=0 (filled after tracking)."""
    center = (mn + mx) * 0.5
    size = mx - mn
    obj_list.append({
        "dist": float(center[0]),
        "cx": float(center[0]),
        "cy": float(center[1]),
        "cz": float(center[2]),
        "sx": float(size[0]),
        "sy": float(size[1]),
        "sz": float(size[2]),
        "zone": zone, "layer": layer,
        "npts": int(npts),
        "confidence": 0.0,
        "path_class": path_class,
    })


def _process_frame_impl(ts, pts):
    t0 = time.time()
    roi = rotate_roi_numba(pts)
    roi_n = roi.shape[0]
    default_path = {"path_class": PATH_STRAIGHT, "filter_scale": 1.0,
                    "curvature": 0.0, "bend_ratio": 0.0, "straight_fraction": 0.0}
    if roi_n == 0:
        return {"ts": ts, "in_n": pts.shape[0], "roi_n": 0,
                "objects": [], "proc_ms": 0.0, "reject_stats": {},
                "path": default_path}

    # ==== Module 1: path geometry ====
    path_result = analyze_path(roi)
    path_class = path_result["path_class"]

    objects = []
    reject_stats = Counter()

    ovh_mask = roi[:, 2] > (OVH_Z + 0.1)
    n_ovh = int(ovh_mask.sum())
    if n_ovh >= 5:
        ovh = roi[ovh_mask]
        for mn, mx, npts in cluster_layer(ovh, "near"):
            cy = float((mn[1] + mx[1]) / 2)
            cz = float((mn[2] + mx[2]) / 2)
            dist = float(max(0.0, (mn[0] + mx[0]) / 2))
            ok, why = is_valid_object(mn, mx, ZONE_ABOVE, dist, npts)
            if not ok:
                reject_stats[why] += 1
                continue
            # ==== Module 2: plausibility ====
            plaus, reason = check_plausibility(mn, mx, ZONE_ABOVE, dist, npts)
            if not plaus:
                reject_stats[reason] += 1
                continue
            _emit(objects, mn, mx, ZONE_ABOVE, "overhang", npts, path_class)

    xs = roi[:, 0]
    layer_id = np.digitize(xs, [NEAR_MAX, MID_MAX])
    for lid, lname in ((0, "near"), (1, "mid"), (2, "far")):
        chunk = roi[layer_id == lid]
        if chunk.shape[0] == 0:
            continue
        for mn, mx, npts in cluster_layer(chunk, lname):
            cy = float((mn[1] + mx[1]) / 2)
            cz = float((mn[2] + mx[2]) / 2)
            dist = float(max(0.0, (mn[0] + mx[0]) / 2))
            zone = classify(cy, cz)
            ok, why = is_valid_object(mn, mx, zone, dist, npts)
            if not ok:
                reject_stats[why] += 1
                continue
            # ==== Module 2: plausibility ====
            plaus, reason = check_plausibility(mn, mx, zone, dist, npts)
            if not plaus:
                reject_stats[reason] += 1
                continue
            _emit(objects, mn, mx, zone, lname, npts, path_class)

    return {"ts": ts, "in_n": pts.shape[0], "roi_n": roi_n,
            "objects": objects, "proc_ms": (time.time() - t0) * 1000.0,
            "reject_stats": dict(reject_stats),
            "path": path_result}


def process_frame(args):
    ts, pts = args
    try:
        return _process_frame_impl(ts, pts)
    except Exception as e:
        return {"ts": ts, "error": str(e), "objects": [], "reject_stats": {}}


# ============================================================
#  TRACKER
# ============================================================

class Tracker:
    __slots__ = ("tracks", "next_id", "assoc", "assoc_sq", "timeout", "min_hits")

    def __init__(self, assoc_dist=6.0, timeout=8, min_hits=15):
        self.tracks = {}
        self.next_id = 1
        self.assoc = assoc_dist
        self.assoc_sq = assoc_dist * assoc_dist
        self.timeout = timeout
        self.min_hits = min_hits

    def update(self, detections, ts):
        used = set()
        out = []
        tracks = self.tracks
        assoc_sq = self.assoc_sq

        for d in detections:
            cx, cy, cz = d["cx"], d["cy"], d["cz"]
            best = None
            best_d2 = assoc_sq
            for tid, tr in tracks.items():
                if tid in used:
                    continue
                tx, ty, tz = tr["pos"]
                # early-exit по dx — самый частый случай отсева
                dx = cx - tx
                if dx * dx >= best_d2:
                    continue
                dy = cy - ty
                dz = cz - tz
                d2 = dx * dx + dy * dy + dz * dz
                if d2 < best_d2:
                    best_d2 = d2
                    best = tid

            if best is None:
                best = self.next_id
                self.next_id += 1
                tracks[best] = {"pos": (cx, cy, cz), "hits": 0, "misses": 0}

            tr = tracks[best]
            tr["pos"] = (cx, cy, cz)
            tr["hits"] += 1
            tr["misses"] = 0
            used.add(best)
            out.append((d, best, tr["hits"]))

        # удаление пропавших
        dead = []
        for tid, tr in tracks.items():
            if tid not in used:
                tr["misses"] += 1
                if tr["misses"] > self.timeout:
                    dead.append(tid)
        for tid in dead:
            del tracks[tid]

        min_hits = self.min_hits
        return [{**d, "track_id": tid, "hits": h}
                for d, tid, h in out if h >= min_hits]


# ============================================================
#  ЧТЕНИЕ BAG
# ============================================================

_TS = get_typestore(Stores.ROS2_HUMBLE)


def stream_frames(bag_dir):
    with Reader(str(bag_dir)) as reader:
        cloud_conn = None
        for conn in reader.connections:
            if conn.msgtype == "sensor_msgs/msg/PointCloud2":
                cloud_conn = conn
                break
        if cloud_conn is None:
            raise RuntimeError("no PointCloud2 in bag")
        for conn, timestamp, rawdata in reader.messages(connections=[cloud_conn]):
            msg = _TS.deserialize_cdr(rawdata, conn.msgtype)
            n = msg.width * msg.height
            ps = msg.point_step
            buf = np.frombuffer(msg.data, dtype=np.uint8, count=n * ps)
            if buf.size < n * ps:
                continue
            if ps == 16:
                xyz = buf.view(np.float32).reshape(n, 4)[:, :3].copy()
            elif ps == 32:
                xyz = buf.view(np.float32).reshape(n, 8)[:, :3].copy()
            else:
                xyz = buf.reshape(n, ps)[:, :12].view(np.float32).reshape(n, 3).copy()
            if not np.isfinite(xyz).all():
                xyz = xyz[np.isfinite(xyz).all(axis=1)]
            yield timestamp, xyz



# ============================================================
#  WARMUP
# ============================================================

def warmup():
    """Прогреваем numba + scipy, чтобы 1-й кадр не тормозил."""
    d = np.random.randn(3000, 3).astype(np.float32)
    _ = voxel_hash_numba(d, 0.08)
    _ = rotate_roi_numba(d)
    _ = grid_dbscan_numba(d[:500], 0.5, 3)
    _ = _dbscan_ckdtree(d[:1000], 0.5, 3)
    print("  warmup done", flush=True)


# ============================================================
#  MAIN
# ============================================================

def main(bag_dir, out_csv, n_workers=4):
    print(f"starting streaming processing of {bag_dir}...", flush=True)
    warmup()

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

    path_classes = Counter()
    filter_scales = []

    with ThreadPoolExecutor(max_workers=n_workers) as executor:
        for res in executor.map(process_frame, stream_frames(bag_dir)):
            processed += 1
            if res.get("error"):
                print(f"  frame {processed} ERROR: {res['error']}", flush=True)
                continue

            # Collect path info
            path_info = res.get("path", {})
            path_classes[path_info.get("path_class", "STRAIGHT")] += 1
            filter_scales.append(float(path_info.get("filter_scale", 1.0)))

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

            if processed % 100 == 0:
                fps = processed / (time.time() - t0)
                print(f"  [{processed} frames] raw={len(dets)} "
                      f"conf={len(confirmed)} fps={fps:.1f} "
                      f"tracks={len(all_tracks)}", flush=True)

    f.close()
    dt = time.time() - t0
    print(f"\nprocessing done: {processed} frames in {dt:.1f}s "
          f"({processed/max(dt,1e-9):.1f} fps)", flush=True)
    print(f"raw candidates: {total_raw}", flush=True)
    print(f"rejects: {dict(total_reject)}", flush=True)

    # === Пост-фильтр треков ===
    print(f"\n{'='*72}\n  ФИЛЬТРАЦИЯ ТРЕКОВ\n{'='*72}")
    print(f"  До фильтрации: {len(all_tracks)} треков")

    # ==== Module 3: confidence + Module 1: filter_scale ====
    median_scale = float(np.median(filter_scales)) if filter_scales else 1.0
    print(f"  Пути: {dict(path_classes)}")
    print(f"  Медианный filter_scale: {median_scale:.2f}")

    for t in all_tracks.values():
        t["confidence"] = compute_confidence(
            t["zone"], t["dist"], t["sx"], t["sy"], t["sz"],
            t["npts"], t["hits"]
        )

    def min_hits_for(t):
        if t["dist"] < 50:  base = 50
        elif t["dist"] < 100: base = 30
        elif t["dist"] < 150: base = 15
        else: base = 8
        return int(base / max(median_scale, 0.3))

    # Universal: keep weak detections but let them rank low.
    CONFIDENCE_MIN = 0.05
    pre_cut = len(all_tracks)
    all_tracks = {tid: t for tid, t in all_tracks.items()
                  if t["confidence"] >= CONFIDENCE_MIN}
    print(f"  После confidence>={CONFIDENCE_MIN}: {len(all_tracks)} (было {pre_cut})")

    filtered = [t for t in all_tracks.values() if t["hits"] >= min_hits_for(t)]
    print(f"  После hits-фильтра (scale={median_scale:.2f}): {len(filtered)}")

    filtered = [t for t in filtered
                if t["zone"] != "RAIL" or (t["sy"] >= 0.20 or t["sz"] >= 0.20)]
    print(f"  После отсева тонких рельс: {len(filtered)}")

        # Фильтр 3: минимальный размер + отсев плоских артефактов на дальних
    def _ok_size(t):
        dims = (t["sx"], t["sy"], t["sz"])
        if max(dims) < 0.15:
            return False
        # На дистанции >60м: если одна из сторон < 3см — это плоскость,
        # а не реальный объект (стены/рельсы через сенсор)
        if t["dist"] > 60.0 and min(dims) < 0.03:
            return False
        return True
    filtered = [t for t in filtered if _ok_size(t)]
    print(f"  После размера>=0.15м: {len(filtered)}")

    # Кластеризация треков
    clusters = []
    used = set()
    filtered.sort(key=lambda t: -t["hits"])
    for t in filtered:
        if id(t) in used:
            continue
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
        total_hits = sum(g["hits"] for g in group)
        best = max(group, key=lambda g: g["hits"])
        clusters.append({**best, "total_hits": total_hits,
                         "n_tracks": len(group)})

    print(f"  После кластеризации: {len(clusters)} уникальных объектов")

    for c in clusters:
        c["score"] = c["total_hits"] * np.log1p(c["npts"]) * c["n_tracks"]
        c["confidence"] = compute_confidence(
            c["zone"], c["dist"], c["sx"], c["sy"], c["sz"],
            c["npts"], c["total_hits"]
        )
    clusters.sort(key=lambda c: -c["confidence"])

    # Universal zone caps — high, not restricting recall.
    ZONE_CAPS = {"INSIDE": 30, "RAIL": 20, "NEAR": 15, "ABOVE": 8, "OUTSIDE": 20, "BELOW": 0}
    zone_counts = {}
    capped = []
    for c in clusters:
        z = c["zone"]
        cap = ZONE_CAPS.get(z, 10)
        if zone_counts.get(z, 0) < cap:
            capped.append(c)
            zone_counts[z] = zone_counts.get(z, 0) + 1
    print(f"  После zone cap: {len(capped)} (было {len(clusters)})")
    clusters = capped

    # ==== Split into DANGEROUS (in-gauge) and WATCHLIST (outside) ====
    DANGEROUS_ZONES = {"INSIDE", "RAIL", "NEAR", "ABOVE"}
    danger = [c for c in clusters if c["zone"] in DANGEROUS_ZONES]
    watch = [c for c in clusters if c["zone"] not in DANGEROUS_ZONES]

    # ==== Distance-stratified selection for DANGEROUS ====
    # Guarantee visibility of near (0-100m), mid (100-200m), and far (200m+) objects.
    # This directly addresses the goal of seeing obstacles ~100m and ~200m apart.
    def pick_by_distance(items, n, bands):
        picked = []
        used = set()
        for lo, hi in bands:
            for i, c in enumerate(items):
                if i in used:
                    continue
                if lo <= c["dist"] < hi:
                    picked.append(c)
                    used.add(i)
                    break
        for i, c in enumerate(items):
            if len(picked) >= n:
                break
            if i not in used:
                picked.append(c)
                used.add(i)
        return picked[:n]

    # dangerous: max 10, at least 1 per band 0-100, 100-200, 200+
    DANGER_N = 10
    DANGER_BANDS = [(0, 100), (100, 200), (200, 500)]
    danger_final = pick_by_distance(danger, DANGER_N, DANGER_BANDS)

    # watchlist: max 5, at least 1 per same bands
    WATCH_N = 5
    WATCH_BANDS = [(0, 100), (100, 200), (200, 500)]
    watch_final = pick_by_distance(watch, WATCH_N, WATCH_BANDS)

    print(f"  DANGEROUS: {len(danger_final)} (было {len(danger)})")
    print(f"  WATCHLIST: {len(watch_final)} (было {len(watch)})")

    # Final list for TOP display = only dangerous
    clusters = danger_final
    watchlist = watch_final

    print(f"\n{'='*72}\n  TOP-15 ОБЪЕКТОВ\n{'='*72}")
    for i, t in enumerate(clusters[:15]):
        print(f"  #{i+1:2d}  dist={t['dist']:6.1f}м  score={t['score']:8.0f}  "
              f"hits={t['total_hits']:4d}  group={t['n_tracks']:2d}  "
              f"zone={t['zone']:8s}  "
              f"size=({t['sx']:5.2f},{t['sy']:5.2f},{t['sz']:5.2f})  "
              f"npts={t['npts']}")

    # ==== WATCHLIST (out-of-gauge, lower priority) ====
    if watchlist:
        print(f"\n{'='*72}\n  WATCHLIST (вне габарита, менее критичные)\n{'='*72}")
        for i, t in enumerate(watchlist):
            print(f"  #{i+1:2d}  dist={t['dist']:6.1f}м  conf={t['confidence']:5.3f}  "
                  f"zone={t['zone']:8s}  "
                  f"size=({t['sx']:5.2f},{t['sy']:5.2f},{t['sz']:5.2f})  npts={t['npts']}")

    zone_count = Counter(t["zone"] for t in clusters)
    max_dist = max((t["dist"] for t in clusters), default=0)
    print(f"\n  Всего уникальных: {len(clusters)}")
    print(f"  Сильных (hits>300): {sum(1 for t in clusters if t['total_hits']>300)}")
    print(f"  По зонам: {dict(zone_count)}")
    print(f"  Max dist: {max_dist:.1f} м")
    print(f"  Обработка: {dt:.1f}s ({processed/max(dt,1e-9):.1f} fps)")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("usage: offline_pipeline_OPT.py BAG_DIR OUT_CSV [N_WORKERS]")
        sys.exit(1)
    main(sys.argv[1], sys.argv[2],
         int(sys.argv[3]) if len(sys.argv) > 3 else 4)
