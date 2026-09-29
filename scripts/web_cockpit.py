#!/usr/bin/env python3
"""Web cockpit — real-time pipeline visualization over WebSocket.

Speed estimation uses ICP (point-to-point) between consecutive frames,
because input bags only contain /lidar_points (no odometry/IMU).
"""
import asyncio, json, math, os, sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import numpy as np
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from scipy.spatial import cKDTree
import uvicorn

from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
from offline_pipeline_v23_FAST import (
    stream_frames, _process_frame_impl, rotate_roi_numba, CRITICAL_ZONES,
    Tracker,
)

app = FastAPI()
STATIC = Path(__file__).parent / "static"

STATE = {
    "running": False, "paused": False, "frame": 0,
    "distance_m": 0.0, "speed_kmh": 0.0, "speed_ema": 0.0,
    "prev_pts": None, "prev_ts": None,
    "t_start": 0.0,
}

MAX_PTS = 10000
ICP_N = 3000
ICP_MIN_RANGE = 3.0
ICP_MAX_RANGE = 60.0


def subsample_arr(roi, n_max=MAX_PTS):
    n = len(roi)
    if n == 0:
        return np.empty((0, 3), dtype=np.float32)
    if n <= n_max:
        return roi.astype(np.float32)
    idx = np.linspace(0, n - 1, n_max).astype(np.int64)
    return roi[idx].astype(np.float32)


def extract_rail_curve(roi, n_bands=20, max_range=220.0):
    """Return polyline [(x, y, z)] approximating the track centreline,
    estimated from near-floor points (|z| < 0.4) in distance bands."""
    if len(roi) < 100:
        return []
    floor = roi[np.abs(roi[:, 2]) < 0.4]
    if len(floor) < 100:
        return []
    xs = floor[:, 0]
    ys = floor[:, 1]
    edges = np.linspace(2.0, max_range, n_bands + 1)
    pts = []
    for i in range(n_bands):
        lo, hi = edges[i], edges[i + 1]
        m = (xs >= lo) & (xs < hi)
        if m.sum() < 20:
            continue
        pts.append((float(xs[m].mean()), float(ys[m].mean()), 0.0))
    return pts

def icp_translation(prev_pts, curr_pts, n=ICP_N, iters=8):
    """Estimate translation of the vehicle between two frames (translation-only ICP).
    Returns (dx, dy, dz) in the LiDAR frame (vehicle moves forward = +X)."""
    if prev_pts is None or curr_pts is None:
        return np.zeros(3, dtype=np.float32)
    if len(prev_pts) < 40 or len(curr_pts) < 40:
        return np.zeros(3, dtype=np.float32)

    def sub(p):
        if len(p) <= n:
            return p
        idx = np.linspace(0, len(p) - 1, n).astype(np.int64)
        return p[idx]

    src = sub(prev_pts)
    dst = sub(curr_pts)

    # Restrict to common forward ROI to avoid near-field/near-edge noise
    m_src = (src[:, 0] > ICP_MIN_RANGE) & (src[:, 0] < ICP_MAX_RANGE)
    m_dst = (dst[:, 0] > ICP_MIN_RANGE) & (dst[:, 0] < ICP_MAX_RANGE)
    src_r = src[m_src].astype(np.float64)
    dst_r = dst[m_dst].astype(np.float64)
    if len(src_r) < 20 or len(dst_r) < 20:
        return np.zeros(3, dtype=np.float32)

    tree = cKDTree(dst_r)
    t = np.zeros(3, dtype=np.float64)

    for _ in range(iters):
        moved = src_r + t
        dists, idxs = tree.query(moved, k=1)
        matched = dst_r[idxs]
        # Weight by inverse distance — trust nearby matches more
        w = 1.0 / (dists + 0.1)
        delta = ((matched - moved) * w[:, None]).sum(0) / w.sum()
        t += delta
        if np.linalg.norm(delta) < 0.005:
            break

    # Scene moved by -t; vehicle moved by +t
    return t.astype(np.float32)


def status_of(dets):
    crit = [d for d in dets if d["zone"] in CRITICAL_ZONES]
    if not crit:
        return "CLEAR", None
    md = min(d["dist"] for d in crit)
    if md < 100:
        return "CRITICAL", md
    if md < 180:
        return "WARNING", md
    return "CLEAR", None


def frame_stream(bag_dir, workers=4):
    with ThreadPoolExecutor(max_workers=workers) as ex:
        q = []
        for i, (ts, pts) in enumerate(stream_frames(bag_dir), 1):
            roi = rotate_roi_numba(pts)
            fut = ex.submit(_process_frame_impl, ts, pts)
            q.append((i, ts, roi, fut))
            if len(q) >= workers + 2:
                idx, ts, roi, f = q.pop(0)
                yield idx, ts, roi, f.result()
        for idx, ts, roi, f in q:
            yield idx, ts, roi, f.result()


async def push_pipeline(ws: WebSocket, bag_dir: str):
    loop = asyncio.get_event_loop()
    gen = frame_stream(bag_dir, workers=4)
    tracker = Tracker(assoc_dist=6.0, timeout=8, min_hits=1)
    last_ts = None

    def nxt():
        try:
            return next(gen)
        except StopIteration:
            return None

    while STATE["running"]:
        if STATE["paused"]:
            await asyncio.sleep(0.05)
            continue

        item = await loop.run_in_executor(None, nxt)
        if item is None:
            break

        idx, ts, roi, res = item
        STATE["frame"] = idx

        dt_s = 0.1
        if last_ts is not None:
            dt_s = max(1e-3, min(0.5, (ts - last_ts) / 1e9))
        last_ts = ts

        # -------- ICP speed estimate --------
        sub = subsample_arr(roi, ICP_N) if roi.shape[0] > 0 else None
        if STATE["prev_pts"] is not None and sub is not None and len(sub) > 20:
            delta_xyz = icp_translation(STATE["prev_pts"], sub)
            dist_m = float(np.linalg.norm(delta_xyz))
            inst_speed = dist_m / dt_s * 3.6      # km/h

            # Outlier clamp
            if inst_speed > 250:
                inst_speed = STATE["speed_ema"]
            if (STATE["speed_ema"] > 5 and
                    abs(inst_speed - STATE["speed_ema"]) > 60):
                inst_speed = STATE["speed_ema"]

            alpha = 0.10
            STATE["speed_ema"] = (1 - alpha) * STATE["speed_ema"] + alpha * inst_speed
            if STATE["speed_ema"] < 0.2:
                STATE["speed_ema"] = 0.0

        STATE["prev_pts"] = sub
        STATE["speed_kmh"] = STATE["speed_ema"]
        STATE["distance_m"] += STATE["speed_kmh"] / 3.6 * dt_s

        # -------- Tracker + telemetry --------
        dets_raw = res.get("objects", [])
        dets_tracked = tracker.update(dets_raw, ts)
        dets = dets_tracked if dets_tracked else dets_raw

        st, md = status_of(dets)
        speed_mps = STATE["speed_kmh"] / 3.6
        ttc = (md / speed_mps) if (md and speed_mps > 0.5) else None

        rail_pts = extract_rail_curve(roi)
        
        payload = {
            "frame": idx,
            "ts_ns": int(ts),
            "points": subsample_arr(roi, MAX_PTS).tolist(),
            "rail_path": rail_pts,
            "detections": [{
                "id": int(d.get("track_id", -1)),
                "zone": d["zone"],
                "dist": float(d["dist"]),
                "cx": float(d["cx"]), "cy": float(d["cy"]), "cz": float(d["cz"]),
                "sx": float(d["sx"]), "sy": float(d["sy"]), "sz": float(d["sz"]),
                "hits": int(d.get("hits", 0)),
            } for d in dets],
            "telemetry": {
                "speed_kmh": round(STATE["speed_kmh"], 1),
                "distance_m": round(STATE["distance_m"], 1),
                "min_dist": md,
                "max_dist": max((d["dist"] for d in dets), default=None),
                "ttc_s": ttc,
                "n_dets": len(dets),
                "n_critical": sum(1 for d in dets if d["zone"] in CRITICAL_ZONES),
                "status": st,
            },
        }
        if idx % 100 == 0:
            try:
                import time as _t
                elapsed = _t.time() - STATE["t_start"] if STATE["t_start"] else 0.0
                fps_now = (idx / elapsed) if elapsed > 0 else 0.0
                n_tracks = len(tracker.tracks)
                print(
                    f"  [{idx:5d} frames] raw={len(dets):3d} conf={len(dets):3d} "
                    f"fps={fps_now:6.1f} tracks={n_tracks:4d} "
                    f"spd={STATE['speed_kmh']:5.2f} km/h "
                    f"dist={STATE['distance_m']:6.1f} m",
                    flush=True,
                )
            except Exception as e:
                print(f"[log err] {e}", flush=True)

        try:
            await ws.send_json(payload)
        except Exception:
            break
        await asyncio.sleep(0.03)

    try:
        import time as _t
        elapsed = _t.time() - STATE["t_start"] if STATE["t_start"] else 0.0
        fps_final = (STATE["frame"] / elapsed) if elapsed > 0 else 0.0
        print()
        print(f"processing done: {STATE['frame']} frames in {elapsed:.1f}s "
              f"({fps_final:.1f} fps)", flush=True)
        print(f"tracks active: {len(tracker.tracks)}", flush=True)
        print(f"speed final: {STATE['speed_kmh']:.2f} km/h", flush=True)
        print(f"path final: {STATE['distance_m']:.1f} m", flush=True)
    except Exception as e:
        print(f"[final log err] {e}", flush=True)


@app.get("/")
async def root():
    return HTMLResponse((STATIC / "index.html").read_text(encoding="utf-8"))


app.mount("/static", StaticFiles(directory=str(STATIC)), name="static")


@app.websocket("/ws")
async def ws_ep(ws: WebSocket):
    await ws.accept()
    pipeline_task = None
    try:
        while True:
            msg = await ws.receive_json()
            cmd = msg.get("cmd")

            if cmd == "start":
                if pipeline_task and not pipeline_task.done():
                    continue
                # Reset state for a fresh run
                STATE["running"] = True
                STATE["paused"] = False
                STATE["frame"] = 0
                STATE["distance_m"] = 0.0
                STATE["speed_kmh"] = 0.0
                STATE["speed_ema"] = 0.0
                STATE["prev_pts"] = None
                STATE["prev_ts"] = None
                import time as _t
                STATE["t_start"] = _t.time()

                bag = msg.get("bag", "bags/cloud_with_fake_obj")
                # Нормализация путей: убираем docker-style /bags/ и /ws/
                PROJECT_ROOT = Path(__file__).resolve().parent.parent
                if bag.startswith("/bags/"):
                    bag = str(PROJECT_ROOT / bag[1:])
                elif bag.startswith("/ws/"):
                    bag = str(PROJECT_ROOT / bag[4:])
                elif not bag.startswith("/"):
                    bag = str(PROJECT_ROOT / bag)
                await ws.send_json({"event": "started", "bag": bag})
                pipeline_task = asyncio.create_task(push_pipeline(ws, bag))

            elif cmd == "pause":
                STATE["paused"] = True
                await ws.send_json({"event": "paused"})

            elif cmd == "resume":
                STATE["paused"] = False
                await ws.send_json({"event": "resumed"})

            elif cmd == "stop":
                STATE["running"] = False
                STATE["paused"] = False
                if pipeline_task and not pipeline_task.done():
                    pipeline_task.cancel()
                await ws.send_json({"event": "stopped"})

    except WebSocketDisconnect:
        STATE["paused"] = True
        STATE["running"] = False
        if pipeline_task and not pipeline_task.done():
            pipeline_task.cancel()


if __name__ == "__main__":
    port = int(os.environ.get("COCKPIT_PORT", "8080"))
    print(f"cockpit on http://0.0.0.0:{port}/", flush=True)
    uvicorn.run(app, host="0.0.0.0", port=port, log_level="warning")
