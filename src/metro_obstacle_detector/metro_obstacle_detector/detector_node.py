#!/usr/bin/env python3
"""Detector для задачи MetroLiDAR.
Многоуровневая обработка по дальности + zone-классификация.
LiDAR 1.075 м над рельсом, рельс z=-1.075 в системе лидара.
"""
import time
import traceback
from dataclasses import dataclass, field
from collections import deque
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from metro_lidar_msgs.msg import (
    Obstacle, ObstacleArray, GabaritViolation, Benchmark,
)

try:
    import open3d as o3d
except ImportError:
    o3d = None


# Зоны относительно габарита
ZONE_INSIDE = "INSIDE"      # внутри габарита — критично
ZONE_NEAR = "NEAR"          # на границе ±0.3 м
ZONE_OUTSIDE = "OUTSIDE"    # рядом, но вне
ZONE_ABOVE = "ABOVE"        # сверху габарита
ZONE_RAIL = "RAIL"          # на уровне рельса
ZONE_BELOW = "BELOW"        # ниже рельса


@dataclass
class Track:
    pos: np.ndarray
    last_time: float
    velocity: float = 0.0
    hits: int = 0
    misses: int = 0
    zone: str = ZONE_INSIDE
    size: np.ndarray = field(default_factory=lambda: np.zeros(3))


class Detector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        # --- Параметры ---
        self.lidar_h = float(self.declare_parameter('lidar_height_m', 1.075).value)
        self.w = float(self.declare_parameter('gabarit_half_width', 1.35).value)
        self.gz_bot = float(self.declare_parameter('gabarit_rail_bottom', 0.20).value)
        self.gz_top = float(self.declare_parameter('gabarit_rail_top', 3.40).value)
        # В системе лидара:
        self.z0 = -self.lidar_h + self.gz_bot        # -0.875
        self.z1 = -self.lidar_h + self.gz_top        # +2.325

        self.fwd_min = float(self.declare_parameter('forward_min', 0.3).value)
        self.fwd_max = float(self.declare_parameter('forward_max', 500.0).value)
        self.lat_lim = float(self.declare_parameter('lateral_limit', 5.0).value)

        self.near_max = float(self.declare_parameter('near_max_m', 50.0).value)
        self.mid_max = float(self.declare_parameter('mid_max_m', 150.0).value)

        self.near_eps = float(self.declare_parameter('near_cluster_eps', 0.30).value)
        self.near_min = int(self.declare_parameter('near_min_points', 3).value)
        self.near_vox = float(self.declare_parameter('near_voxel', 0.03).value)

        self.mid_eps = float(self.declare_parameter('mid_cluster_eps', 0.60).value)
        self.mid_min = int(self.declare_parameter('mid_min_points', 4).value)
        self.mid_vox = float(self.declare_parameter('mid_voxel', 0.08).value)

        self.far_eps = float(self.declare_parameter('far_cluster_eps', 2.0).value)
        self.far_min = int(self.declare_parameter('far_min_points', 3).value)
        self.far_vox = float(self.declare_parameter('far_voxel', 0.25).value)

        self.ovh_z = float(self.declare_parameter('overhang_z_min', 2.325).value)
        self.ovh_min = int(self.declare_parameter('overhang_min_points', 5).value)
        self.ovh_eps = float(self.declare_parameter('overhang_cluster_eps', 1.5).value)

        self.confirm_n = int(self.declare_parameter('confirm_frames', 2).value)
        self.assoc = float(self.declare_parameter('association_distance', 10.0).value)
        self.timeout = int(self.declare_parameter('track_timeout_frames', 12).value)
        self.min_hits = int(self.declare_parameter('min_track_hits_for_report', 2).value)

        self.critical = float(self.declare_parameter('critical_distance', 100.0).value)
        self.warning = float(self.declare_parameter('warning_distance', 300.0).value)
        self.far_d = float(self.declare_parameter('far_distance', 500.0).value)

        self.conf_min = float(self.declare_parameter('confidence_min', 0.08).value)
        self.publish_unconf = bool(self.declare_parameter('publish_unconfirmed', True).value)

        # --- Состояние ---
        self.tracks = {}
        self.next_id = 1
        self.frame_cnt = 0
        self.frame_times = deque(maxlen=30)

        # --- QoS ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.sub = self.create_subscription(
            PointCloud2, '/preprocessing/points', self.cb, qos)
        self.pub = self.create_publisher(ObstacleArray, '/obstacle_detection', qos)
        self.vpub = self.create_publisher(GabaritViolation, '/gabarit/violations', qos)
        self.mpub = self.create_publisher(MarkerArray, '/obstacle_markers', qos)
        self.bpub = self.create_publisher(Benchmark, '/benchmark', qos)

        self.get_logger().info(
            f"detector: lidar_h={self.lidar_h} z_gabarit=[{self.z0:.3f},{self.z1:.3f}] "
            f"fwd=[{self.fwd_min},{self.fwd_max}] zones=6")

    # ------------------------------------------------------------------
    # Классификация зон
    # ------------------------------------------------------------------
    def classify(self, cy: float, cz: float) -> str:
        """Определить зону объекта относительно габарита."""
        ay = abs(cy)
        # на уровне рельса (z около -lidar_h)
        if cz < -self.lidar_h + 0.15 and cz > -self.lidar_h - 0.30:
            return ZONE_RAIL
        if cz < -self.lidar_h:
            return ZONE_BELOW
        if cz > self.z1:
            return ZONE_ABOVE
        if ay <= self.w and self.z0 <= cz <= self.z1:
            return ZONE_INSIDE
        # Расстояние до границ
        dy = ay - self.w
        dz_top = cz - self.z1
        dz_bot = self.z0 - cz
        if 0 < dy <= 0.30 or 0 < dz_top <= 0.30 or 0 < dz_bot <= 0.30:
            return ZONE_NEAR
        return ZONE_OUTSIDE

    # ------------------------------------------------------------------
    # Кластеризация адаптивная по дальности
    # ------------------------------------------------------------------
    def cluster_layer(self, pts: np.ndarray, layer: str) -> List[np.ndarray]:
        if pts.shape[0] == 0:
            return []
        if layer == "near":
            eps, minp, vox = self.near_eps, self.near_min, self.near_vox
        elif layer == "mid":
            eps, minp, vox = self.mid_eps, self.mid_min, self.mid_vox
        else:
            eps, minp, vox = self.far_eps, self.far_min, self.far_vox

        # Voxel-дедупликация
        if vox > 0:
            keys = np.floor(pts / vox).astype(np.int64)
            _, idx = np.unique(keys, axis=0, return_index=True)
            pts = pts[np.sort(idx)]

        if pts.shape[0] < minp:
            return []

        if o3d is None:
            return self._grid_cluster(pts, eps, minp)

        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts.astype(np.float64)))
        labels = np.asarray(pc.cluster_dbscan(
            eps=eps, min_points=minp, print_progress=False))
        result = []
        for lbl in np.unique(labels):
            if lbl < 0:
                continue
            c = pts[labels == lbl]
            if c.shape[0] >= minp:
                result.append(c)
        return result

    def _grid_cluster(self, pts, eps, minp):
        cells = np.floor(pts[:, :2] / eps).astype(np.int32)
        buckets = {}
        for i, k in enumerate(map(tuple, cells)):
            buckets.setdefault(k, []).append(i)
        return [pts[v] for v in buckets.values() if len(v) >= minp]

    # ------------------------------------------------------------------
    # Основной callback
    # ------------------------------------------------------------------
    def cb(self, msg):
        self.frame_cnt += 1
        try:
            self._cb_impl(msg)
        except Exception as e:
            self.get_logger().error(f"cb exc: {e}\n{traceback.format_exc()}")

    def _cb_impl(self, msg):
        t0 = time.perf_counter()
        now = time.time()

        arr = point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if arr.size == 0:
            return
        pts = np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(np.float32)

        # Поворот: forward -> +X, lateral -> Y
        x_old = pts[:, 0].copy()
        y_old = pts[:, 1].copy()
        pts[:, 0] = -y_old
        pts[:, 1] = x_old

        # ROI: forward, lateral, z
        x = pts[:, 0]
        m = (x >= self.fwd_min) & (x <= self.fwd_max)
        m &= np.abs(pts[:, 1]) <= self.lat_lim
        m &= pts[:, 2] >= self.z0 - 1.0
        m &= pts[:, 2] <= self.z1 + 1.5
        roi = pts[m]

        # --- Двухслойная обработка ---
        objs = []  # (cluster, layer, zone)

        # Свисающие сверху — отдельно
        overhang = roi[roi[:, 2] > self.ovh_z]
        if overhang.shape[0] >= self.ovh_min:
            for c in self.cluster_layer(overhang, "near"):
                objs.append((c, "overhang", ZONE_ABOVE))

        # Основной поток по слоям
        near = roi[(roi[:, 0] >= self.fwd_min) & (roi[:, 0] < self.near_max)]
        mid = roi[(roi[:, 0] >= self.near_max) & (roi[:, 0] < self.mid_max)]
        far = roi[(roi[:, 0] >= self.mid_max) & (roi[:, 0] <= self.fwd_max)]

        for layer, chunk in (("near", near), ("mid", mid), ("far", far)):
            for c in self.cluster_layer(chunk, layer):
                mn = c.min(0); mx = c.max(0)
                cy = float((mn[1] + mx[1]) / 2)
                cz = float((mn[2] + mx[2]) / 2)
                zone = self.classify(cy, cz)
                objs.append((c, layer, zone))

        # --- Формируем detections ---
        detections = []
        for c, layer, zone in objs:
            mn = c.min(0); mx = c.max(0)
            center = (mn + mx) / 2.0
            dist = float(max(0.0, center[0]))
            size = mx - mn
            detections.append({
                "c": c, "mn": mn, "mx": mx, "center": center,
                "dist": dist, "size": size, "zone": zone,
                "layer": layer, "npts": int(c.shape[0]),
            })

        # --- Tracking ---
        assigned = self.associate(detections, now)

        # --- Публикация ---
        out = ObstacleArray()
        out.header = msg.header
        out.point_count = len(pts)
        out.candidate_count = len(detections)
        markers = MarkerArray()

        published = 0
        for i, d in enumerate(detections):
            oid = assigned[i]
            tr = self.tracks[oid]
            confirmed = tr.hits >= self.confirm_n
            if not confirmed and tr.hits < self.min_hits:
                continue

            o = Obstacle()
            o.header = msg.header
            o.id = oid
            o.detected = True
            o.confirmed = confirmed
            o.inside_gabarit = (d["zone"] == ZONE_INSIDE)
            o.risk_level = self.risk(d["dist"], d["zone"])
            o.confidence = self.confidence(d, tr)
            o.distance_m = d["dist"]
            o.velocity_mps = float(tr.velocity)
            o.ttc_s = (max(0.0, d["dist"] / (-tr.velocity))
                       if tr.velocity < -0.05 else -1.0)
            o.position.x = float(d["center"][0])
            o.position.y = float(d["center"][1])
            o.position.z = float(d["center"][2])
            o.size.x = float(max(d["size"][0], 0.05))
            o.size.y = float(max(d["size"][1], 0.05))
            o.size.z = float(max(d["size"][2], 0.05))
            o.gabarit_margin_m = self.margin(d["center"])
            o.class_name = d["zone"]  # используем поле как zone-label
            out.obstacles.append(o)
            published += 1

            mk = self._marker(msg.header, oid, d, confirmed)
            markers.markers.append(mk)

            if d["zone"] in (ZONE_INSIDE, ZONE_NEAR):
                v = GabaritViolation()
                v.header = msg.header
                v.violation = True
                v.obstacle_id = oid
                v.min_distance_to_gabarit_m = max(0.0, -self.margin(d["center"]))
                v.risk_level = o.risk_level
                self.vpub.publish(v)

        elapsed = (time.perf_counter() - t0) * 1000.0
        out.processing_time_ms = elapsed

        self.pub.publish(out)
        self.mpub.publish(markers)

        if self.frame_cnt % 5 == 0 or published:
            dists = sorted(round(o.distance_m, 1) for o in out.obstacles)
            zones = [o.class_name for o in out.obstacles]
            self.get_logger().info(
                f"F{self.frame_cnt} in={arr.size} roi={roi.shape[0]} "
                f"cand={len(detections)} pub={published} "
                f"dists={dists[:8]} zones={zones[:8]} t={elapsed:.0f}ms")

    # ------------------------------------------------------------------
    # Вспомогательные
    # ------------------------------------------------------------------
    def margin(self, center: np.ndarray) -> float:
        """Зазор до границы габарита (отриц. — внутри)."""
        cy, cz = float(center[1]), float(center[2])
        my = self.w - abs(cy)
        mz_top = self.z1 - cz
        mz_bot = cz - self.z0
        return min(my, mz_top, mz_bot)

    def risk(self, dist: float, zone: str) -> int:
        if zone in (ZONE_INSIDE, ZONE_RAIL) and dist <= self.critical:
            return 4
        if zone == ZONE_NEAR and dist <= self.critical:
            return 3
        if zone in (ZONE_INSIDE, ZONE_NEAR, ZONE_ABOVE):
            return 3
        if dist <= self.warning:
            return 2
        return 1

    def confidence(self, d, tr) -> float:
        npts = d["npts"]
        dist = d["dist"]
        density = min(1.0, npts / 30.0)
        range_score = max(0.0, 1.0 - dist / max(self.fwd_max, 1.0))
        temporal = min(1.0, tr.hits / max(self.confirm_n, 1))
        zone_score = {ZONE_INSIDE: 1.0, ZONE_RAIL: 0.9, ZONE_NEAR: 0.7,
                      ZONE_ABOVE: 0.7, ZONE_OUTSIDE: 0.4, ZONE_BELOW: 0.3}.get(d["zone"], 0.5)
        return float(np.clip(
            0.10 + 0.30 * density + 0.20 * range_score +
            0.20 * temporal + 0.20 * zone_score, 0, 0.99))

    def associate(self, dets, now):
        used = set()
        ids = []
        for d in dets:
            c = d["center"]
            best = None
            bd = self.assoc
            for oid, tr in self.tracks.items():
                if oid in used:
                    continue
                dist = float(np.linalg.norm(c - tr.pos))
                if dist < bd:
                    best = oid
                    bd = dist
            if best is None:
                best = self.next_id
                self.next_id += 1
                self.tracks[best] = Track(c.copy(), now)
            tr = self.tracks[best]
            dt = max(now - tr.last_time, 1e-3)
            vx = float((c[0] - tr.pos[0]) / dt)
            tr.velocity = 0.6 * tr.velocity + 0.4 * vx
            tr.pos = c.copy()
            tr.last_time = now
            tr.hits += 1
            tr.misses = 0
            tr.zone = d["zone"]
            tr.size = d["size"]
            used.add(best)
            ids.append(best)

        for oid, tr in list(self.tracks.items()):
            if oid not in used:
                tr.misses += 1
            if tr.misses > self.timeout:
                del self.tracks[oid]
        return ids

    def _marker(self, header, oid, d, confirmed):
        from geometry_msgs.msg import Point as P
        mk = Marker()
        mk.header = header
        mk.ns = "obstacles"
        mk.id = int(oid)
        mk.type = Marker.CUBE
        mk.action = Marker.ADD
        mk.pose.position.x = float(d["center"][0])
        mk.pose.position.y = float(d["center"][1])
        mk.pose.position.z = float(d["center"][2])
        mk.scale.x = float(max(d["size"][0], 0.05))
        mk.scale.y = float(max(d["size"][1], 0.05))
        mk.scale.z = float(max(d["size"][2], 0.05))
        colors = {
            ZONE_INSIDE: (1.0, 0.05, 0.05),
            ZONE_RAIL: (1.0, 0.4, 0.05),
            ZONE_NEAR: (1.0, 0.75, 0.1),
            ZONE_ABOVE: (0.5, 0.2, 1.0),
            ZONE_OUTSIDE: (0.6, 0.6, 0.6),
            ZONE_BELOW: (0.3, 0.3, 0.8),
        }
        c = colors.get(d["zone"], (0.7, 0.7, 0.7))
        mk.color = ColorRGBA(r=c[0], g=c[1], b=c[2], a=0.85 if confirmed else 0.4)
        return mk


def main():
    rclpy.init()
    node = Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()