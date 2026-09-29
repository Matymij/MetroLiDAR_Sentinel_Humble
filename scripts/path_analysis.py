#!/usr/bin/env python3
"""Path geometry analysis for metro tunnel.

Estimates rail centerline curvature from LiDAR floor points and classifies
each forward distance band as STRAIGHT, GENTLE_TURN, or SHARP_TURN.

Used by the pipeline to adapt detection filters: on straight segments we trust
far objects (walls are parallel to gauge, false positives unlikely); on turns
we tighten thresholds because tunnel walls enter the gauge geometrically.
"""
import numpy as np
from numba import njit

# Curvature thresholds (1/m). 1/500 = 500 m radius = gentle, 1/200 = sharp
CURV_GENTLE = 1.0 / 500.0
CURV_SHARP = 1.0 / 200.0

# Path classes
PATH_STRAIGHT = "STRAIGHT"
PATH_GENTLE = "GENTLE_TURN"
PATH_SHARP = "SHARP_TURN"


@njit(cache=True, fastmath=True, nogil=True)
def _bin_floor_points(roi, n_bands, max_range):
    """Bin floor points (|z|<0.4) by forward distance.

    Returns (mean_x, mean_y, count) arrays of length n_bands.
    """
    n = roi.shape[0]
    sum_x = np.zeros(n_bands, dtype=np.float64)
    sum_y = np.zeros(n_bands, dtype=np.float64)
    cnt = np.zeros(n_bands, dtype=np.int32)

    band_width = max_range / n_bands
    for i in range(n):
        z = roi[i, 2]
        if z < -0.4 or z > 0.4:
            continue
        x = roi[i, 0]
        if x < 2.0 or x > max_range:
            continue
        b = int(x / band_width)
        if b < 0 or b >= n_bands:
            continue
        sum_x[b] += x
        sum_y[b] += roi[i, 1]
        cnt[b] += 1

    mean_x = np.zeros(n_bands, dtype=np.float32)
    mean_y = np.zeros(n_bands, dtype=np.float32)
    for b in range(n_bands):
        if cnt[b] >= 20:
            mean_x[b] = sum_x[b] / cnt[b]
            mean_y[b] = sum_y[b] / cnt[b]
    return mean_x, mean_y, cnt


@njit(cache=True, fastmath=True, nogil=True)
def _polyfit_curvature(xs, ys, n_valid):
    """Fit y = a*x^2 + b*x + c via least squares, return |a| as curvature proxy.

    We use a quadratic fit; coefficient a is proportional to curvature.
    """
    n = xs.shape[0]
    if n_valid < 4:
        return 0.0
    # Build normal equations for [x^2, x, 1] basis
    S0 = 0.0; S1 = 0.0; S2 = 0.0; S3 = 0.0; S4 = 0.0
    T0 = 0.0; T1 = 0.0; T2 = 0.0
    for i in range(n):
        if xs[i] == 0.0 and ys[i] == 0.0:
            continue
        xi = xs[i]; yi = ys[i]
        xi2 = xi * xi
        S0 += 1.0
        S1 += xi
        S2 += xi2
        S3 += xi2 * xi
        S4 += xi2 * xi2
        T0 += yi
        T1 += xi * yi
        T2 += xi2 * yi

    # Solve 3x3 system for [a, b, c]
    # |S4 S3 S2| |a|   |T2|
    # |S3 S2 S1| |b| = |T1|
    # |S2 S1 S0| |c|   |T0|
    A = np.array([[S4, S3, S2], [S3, S2, S1], [S2, S1, S0]], dtype=np.float64)
    B = np.array([T2, T1, T0], dtype=np.float64)
    try:
        det = (A[0, 0] * (A[1, 1] * A[2, 2] - A[1, 2] * A[2, 1])
               - A[0, 1] * (A[1, 0] * A[2, 2] - A[1, 2] * A[2, 0])
               + A[0, 2] * (A[1, 0] * A[2, 1] - A[1, 1] * A[2, 0]))
        if abs(det) < 1e-12:
            return 0.0
        # Cramer's rule
        Aa = np.array([[B[0], A[0, 1], A[0, 2]],
                       [B[1], A[1, 1], A[1, 2]],
                       [B[2], A[2, 1], A[2, 2]]], dtype=np.float64)
        det_a = (Aa[0, 0] * (Aa[1, 1] * Aa[2, 2] - Aa[1, 2] * Aa[2, 1])
                 - Aa[0, 1] * (Aa[1, 0] * Aa[2, 2] - Aa[1, 2] * Aa[2, 0])
                 + Aa[0, 2] * (Aa[1, 0] * Aa[2, 1] - Aa[1, 1] * Aa[2, 0]))
        a = det_a / det
        return abs(a)
    except Exception:
        return 0.0


def analyze_path(roi, n_bands=20, max_range=150.0):
    """Analyze path geometry from ROI points.

    Returns a dict:
        {
            "curvature": float,          # fitted quadratic coefficient (1/m)
            "path_class": str,           # STRAIGHT / GENTLE_TURN / SHARP_TURN
            "bend_ratio": float,         # |y_max - y_min| / x_span
            "filter_scale": float,       # multiplier for detection thresholds
            "straight_fraction": float,  # fraction of bands with valid floor
        }
    """
    if roi.shape[0] < 100:
        return _default_result()

    mean_x, mean_y, cnt = _bin_floor_points(roi, n_bands, max_range)
    valid = cnt >= 20
    n_valid = int(valid.sum())
    if n_valid < 4:
        return _default_result()

    xs = mean_x[valid]
    ys = mean_y[valid]

    curv = _polyfit_curvature(xs, ys, n_valid)

    # Bend ratio: how much lateral deviation
    y_span = float(ys.max() - ys.min())
    x_span = float(xs.max() - xs.min())
    bend_ratio = y_span / max(x_span, 1.0)

    # Classify
    if curv >= CURV_SHARP:
        path_class = PATH_SHARP
        filter_scale = 0.5      # aggressive filtering (smaller objects need more hits)
    elif curv >= CURV_GENTLE:
        path_class = PATH_GENTLE
        filter_scale = 0.75
    else:
        path_class = PATH_STRAIGHT
        filter_scale = 1.0      # normal filtering, trust detections

    straight_fraction = n_valid / n_bands

    return {
        "curvature": float(curv),
        "path_class": path_class,
        "bend_ratio": bend_ratio,
        "filter_scale": filter_scale,
        "straight_fraction": float(straight_fraction),
    }


def _default_result():
    return {
        "curvature": 0.0,
        "path_class": PATH_STRAIGHT,
        "bend_ratio": 0.0,
        "filter_scale": 1.0,
        "straight_fraction": 0.0,
    }
