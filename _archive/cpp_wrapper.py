"""ctypes wrapper for cpp_fast.so (C ABI). GIL auto-released."""
import ctypes
import os
import numpy as np

_LIB_PATH = os.environ.get("CPP_FAST_LIB", "/tmp/cpp_build/cpp_fast.so")
_lib = None
_available = False

try:
    if os.path.isfile(_LIB_PATH):
        _lib = ctypes.CDLL(_LIB_PATH)
        _lib.rotate_roi_c.restype = ctypes.c_long
        _lib.rotate_roi_c.argtypes = [
            ctypes.POINTER(ctypes.c_float), ctypes.c_long,
            ctypes.c_float, ctypes.c_float, ctypes.c_float,
            ctypes.c_float, ctypes.c_float,
            ctypes.POINTER(ctypes.c_float),
        ]
        _lib.voxel_hash_c.restype = ctypes.c_long
        _lib.voxel_hash_c.argtypes = [
            ctypes.POINTER(ctypes.c_float), ctypes.c_long,
            ctypes.c_float,
            ctypes.POINTER(ctypes.c_long),
        ]
        _lib.grid_dbscan_c.restype = ctypes.c_long
        _lib.grid_dbscan_c.argtypes = [
            ctypes.POINTER(ctypes.c_float), ctypes.c_long,
            ctypes.c_float, ctypes.c_int,
            ctypes.POINTER(ctypes.c_int),
        ]
        _available = True
        print(f"[CPP] loaded {_LIB_PATH}", flush=True)
    else:
        print(f"[CPP] not found: {_LIB_PATH}", flush=True)
except Exception as e:
    print(f"[CPP] load error: {e}", flush=True)
    _lib = None
    _available = False


def is_available():
    return _available


def _fptr(arr):
    return arr.ctypes.data_as(ctypes.POINTER(ctypes.c_float))


def rotate_roi(pts, fwd_min, fwd_max, lat_lim, z_min, z_max):
    pts = np.ascontiguousarray(pts, dtype=np.float32)
    n = pts.shape[0]
    out = np.empty((n, 3), dtype=np.float32)
    m = _lib.rotate_roi_c(
        _fptr(pts), ctypes.c_long(n),
        ctypes.c_float(fwd_min), ctypes.c_float(fwd_max),
        ctypes.c_float(lat_lim),
        ctypes.c_float(z_min), ctypes.c_float(z_max),
        _fptr(out),
    )
    return out[:m]


def voxel_hash(pts, vox):
    pts = np.ascontiguousarray(pts, dtype=np.float32)
    n = pts.shape[0]
    out = np.empty(n, dtype=np.int64)
    m = _lib.voxel_hash_c(
        _fptr(pts), ctypes.c_long(n),
        ctypes.c_float(vox),
        out.ctypes.data_as(ctypes.POINTER(ctypes.c_long)),
    )
    return out[:m]


def grid_dbscan(pts, eps, min_pts):
    pts = np.ascontiguousarray(pts, dtype=np.float32)
    n = pts.shape[0]
    out = np.empty(n, dtype=np.int32)
    _lib.grid_dbscan_c(
        _fptr(pts), ctypes.c_long(n),
        ctypes.c_float(eps), ctypes.c_int(min_pts),
        out.ctypes.data_as(ctypes.POINTER(ctypes.c_int)),
    )
    return out