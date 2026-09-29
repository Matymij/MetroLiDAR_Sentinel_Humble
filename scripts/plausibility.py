#!/usr/bin/env python3
"""Plausibility classifier for metro obstacle detection.

Rejects physically impossible detections: flat walls masquerading as objects,
cables in mid-air, ceiling fragments, and asymmetric noise clusters.

This module is intentionally rule-based (no ML) so it stays explainable and
requires no training data. Rules derived from:
- Pandar128E3X beam divergence (0.125° vertical → ~2 cm at 100 m)
- Metro gauge geometry (1.35 m half-width, 3.4 m height above rail)
- Physical properties of real obstacles (min 5 cm thickness)
"""
import numpy as np

# Physical limits
MIN_THICKNESS_M = 0.03          # anything thinner than 5 cm = flat surface
MIN_THICKNESS_FAR_M = 0.10      # at >60 m, need 20 cm to be a real obstacle
MAX_OBJECT_HEIGHT_M = 4.0       # above this = ceiling structure
MIN_OBJECT_HEIGHT_M = 0.10      # below this = floor noise
CABLE_SY_THRESHOLD_M = 0.10     # hanging cable: narrow in lateral (y)
CABLE_CZ_THRESHOLD_M = 2.20     # hanging cable: above 2.2 m
FLAT_ASPECT_THRESHOLD = 35.0    # max/min dimension ratio

# Reason codes returned to caller
REASON_OK = "ok"
REASON_TOO_THIN = "plaus_too_thin"
REASON_TOO_FLAT = "plaus_too_flat"
REASON_CEILING = "plaus_ceiling"
REASON_FLOOR_NOISE = "plaus_floor_noise"
REASON_HANGING_CABLE = "plaus_hanging_cable"
REASON_GHOST_NEAR = "plaus_ghost_near"
REASON_GHOST_FAR = "plaus_ghost_far"
REASON_LATERAL_WALL = "plaus_lateral_wall"
REASON_HORIZONTAL_SLAB = "plaus_horizontal_slab"


def check_plausibility(mn, mx, zone, dist, npts):
    """Return (is_plausible: bool, reason: str).

    Args:
        mn: (3,) min corner of bbox
        mx: (3,) max corner of bbox
        zone: zone string (INSIDE/RAIL/etc.)
        dist: forward distance to object center (m)
        npts: number of points in the cluster
    """
    size = mx - mn
    sx, sy, sz = float(size[0]), float(size[1]), float(size[2])

    dims = sorted([sx, sy, sz])
    mind = dims[0]
    midd = dims[1]
    maxd = dims[2]
    aspect = maxd / max(mind, 1e-6)

    cz_center = float((mn[2] + mx[2]) / 2.0)

    # 1a. Ceiling: object above rail envelope (Z1 = 3.4 m) AND wide horizontally.
    #     A tunnel ceiling fragment is a wide flat structure at 4+ m above rail.
    #     Real obstacles above the gauge are narrow (pipes, cables, fixtures).
    GAUGE_TOP_M = 3.40
    CEILING_WIDTH_M = 3.0
    if cz_center > GAUGE_TOP_M + 0.3 and max(sx, sy) > CEILING_WIDTH_M:
        return False, REASON_CEILING

    # 1b. Absolute size cap: any structure taller than 4 m -> ceiling
    if maxd > MAX_OBJECT_HEIGHT_M and sz > MAX_OBJECT_HEIGHT_M:
        return False, REASON_CEILING

    # 2. Floor noise: object entirely below 10 cm and very thin
    if cz_center < MIN_OBJECT_HEIGHT_M and sz < MIN_OBJECT_HEIGHT_M:
        return False, REASON_FLOOR_NOISE

    # 3a. Absolute minimum: mind < 3 cm is always an artifact (flat wall slice).
    #     Even at close range, real LiDAR cluster has some 3D extent.
    if mind < 0.03:
        return False, REASON_TOO_THIN

    # 3b. Middle dimension must exceed adaptive threshold.
    #     Real objects have at least two substantial dimensions; a single thin
    #     side (>= 3 cm) is often caused by beam resolution or outlier points.
    min_thickness = MIN_THICKNESS_FAR_M if dist > 60.0 else MIN_THICKNESS_M
    if midd < min_thickness:
        return False, REASON_TOO_THIN

    # 4. Flat wall/cable/plane: extreme aspect ratio
    if aspect > FLAT_ASPECT_THRESHOLD:
        return False, REASON_TOO_FLAT

    # 5. Hanging cable: narrow in y, high above floor
    if sy < CABLE_SY_THRESHOLD_M and cz_center > CABLE_CZ_THRESHOLD_M:
        return False, REASON_HANGING_CABLE

    # 6. Ghost detections in low zones:
    #    In RAIL/BELOW zones near the sensor, expect solid ground noise.
    #    A non-RAIL zone object very close to sensor (<5 m) with <5 points
    #    is almost always ghost.
    if dist < 5.0 and npts < 5 and zone not in ("RAIL", "NEAR"):
        return False, REASON_GHOST_NEAR

    # 6b. Lateral wall slice: wide in y, thin in x. Real fake objects are
    #     cubes (all dims similar) or long-flat (sx dominant) or thin-hanging
    #     (sz dominant). A wall slice has sy dominant.
    if sy > 1.2 and sy > 2.0 * max(sx, sz):
        return False, REASON_LATERAL_WALL

    # 6c. Horizontal slab: very thin in z, wide in x AND y. Ceiling/floor slice.
    if sz < 0.10 and sx > 1.0 and sy > 1.0:
        return False, REASON_HORIZONTAL_SLAB

    # 7. Ghost far-away: sparse cluster beyond 150 m AND small bbox.
    #    A 2+ m object can legitimately give only 3-4 points at 180 m;
    #    a tiny cluster with 3 points is more likely an artifact.
    if dist > 150.0 and npts < 3 and maxd < 0.5:
        return False, REASON_GHOST_FAR

    return True, REASON_OK


def plausibility_stats_batch(detections):
    """Run plausibility on a list of detection dicts.

    Returns (kept, rejected_reasons_counter).
    Each detection is expected to have mn, mx, zone, dist, npts keys.
    """
    from collections import Counter
    reasons = Counter()
    kept = []
    for d in detections:
        ok, reason = check_plausibility(
            d["mn"], d["mx"], d["zone"], d["dist"], d["npts"]
        )
        if ok:
            kept.append(d)
        else:
            reasons[reason] += 1
    return kept, reasons
