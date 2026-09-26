#!/usr/bin/env python3
"""Потоковая обработка new_data.zst — извлекаем по одному .db3, обрабатываем, удаляем.
Пик на диске: 0.5 ГБ. Работает без metadata.yaml через sqlite3 напрямую.
"""
import csv
import os
import shutil
import sqlite3
import sys
import tarfile
import time
from collections import Counter

import numpy as np
import zstandard
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, "/ws/scripts")
from offline_pipeline import _process_frame_impl, Tracker


def read_frames_from_db3(db3_path):
    """Прямое чтение sqlite ROS2 bag без metadata.yaml."""
    conn = sqlite3.connect(db3_path)
    try:
        topics = {}
        for tid, name, ttype in conn.execute(
                "SELECT id, name, type FROM topics"):
            if ttype == "sensor_msgs/msg/PointCloud2":
                topics[tid] = name
        if not topics:
            return
        for tid in topics:
            cur = conn.execute(
                "SELECT timestamp, data FROM messages "
                "WHERE topic_id=? ORDER BY timestamp", (tid,))
            for ts, data in cur:
                try:
                    msg = deserialize_message(data, PointCloud2)
                except Exception:
                    continue
                if msg.width == 0 or msg.height == 0:
                    continue
                n = msg.width * msg.height
                ps = msg.point_step
                if ps < 12:
                    continue
                buf = np.frombuffer(msg.data, dtype=np.uint8, count=n * ps)
                if len(buf) < n * ps:
                    continue
                xyz = buf.reshape(n, ps)[:, :12].view(
                    np.float32).reshape(n, 3).copy()
                finite = np.isfinite(xyz).all(axis=1)
                if not finite.all():
                    xyz = xyz[finite]
                yield ts, xyz
    finally:
        conn.close()


def main(zst_path, out_csv):
    print(f"streaming {zst_path}", flush=True)
    tmp_db3 = "/tmp/current_stream.db3"
    tracker = Tracker(assoc_dist=6.0, timeout=8, min_hits=15)
    all_tracks = {}
    total_reject = Counter()
    total_raw = 0
    total_files = 0
    total_frames = 0
    t_start = time.time()

    f = open(out_csv, "w", newline="")
    w = csv.writer(f)
    w.writerow(["file", "frame", "ts_ns", "in_points", "roi_points",
                "raw_candidates", "confirmed", "max_dist",
                "dist_list", "zone_list", "size_list", "track_id_list"])

    with open(zst_path, "rb") as fz:
        d = zstandard.ZstdDecompressor()
        with d.stream_reader(fz) as r:
            with tarfile.open(fileobj=r, mode="r|") as tar:
                for member in tar:
                    if not member.isfile():
                        continue
                    if not member.name.endswith(".db3"):
                        continue

                    total_files += 1
                    fname = os.path.basename(member.name)

                    # Извлечь
                    t0 = time.time()
                    extracted = tar.extractfile(member)
                    if extracted is None:
                        continue
                    with open(tmp_db3, "wb") as out:
                        shutil.copyfileobj(extracted, out, length=1 << 20)
                    extract_s = time.time() - t0

                    print(f"\n[{total_files}] {fname} "
                          f"({member.size/1e6:.0f} MB, extract {extract_s:.1f}s)",
                          flush=True)

                    # Обработать
                    t0 = time.time()
                    file_frames = 0
                    for ts, pts in read_frames_from_db3(tmp_db3):
                        total_frames += 1
                        file_frames += 1
                        res = _process_frame_impl(ts, pts)
                        dets = res.get("objects", [])
                        total_raw += len(dets)
                        for k, v in res.get("reject_stats", {}).items():
                            total_reject[k] += v

                        confirmed = tracker.update(dets, ts)
                        for dd in confirmed:
                            tid = dd["track_id"]
                            if (tid not in all_tracks or
                                    dd["hits"] > all_tracks[tid]["hits"]):
                                all_tracks[tid] = dd

                        dists = sorted(round(dd["dist"], 1) for dd in confirmed)
                        zones = [dd["zone"] for dd in confirmed]
                        sizes = [[round(dd["sx"], 2), round(dd["sy"], 2),
                                  round(dd["sz"], 2)] for dd in confirmed]
                        tids = [dd["track_id"] for dd in confirmed]
                        max_d = max(dists) if dists else 0.0

                        w.writerow([fname, file_frames, ts,
                                    res.get("in_n", 0), res.get("roi_n", 0),
                                    len(dets), len(confirmed),
                                    round(max_d, 1), dists, zones,
                                    sizes, tids])

                    process_s = time.time() - t0
                    os.remove(tmp_db3)
                    print(f"  {file_frames} frames | process {process_s:.1f}s | "
                          f"tracks={len(all_tracks)} | files done={total_files}",
                          flush=True)

    f.close()
    total_s = time.time() - t_start

    print(f"\n{'='*72}")
    print(f"  NEW_DATA РЕЗУЛЬТАТЫ")
    print(f"{'='*72}")
    print(f"  Файлов: {total_files}")
    print(f"  Кадров: {total_frames}")
    print(f"  Raw candidates: {total_raw}")
    print(f"  Rejects: {dict(total_reject)}")
    print(f"  Время: {total_s:.1f}s ({total_frames/max(total_s,1):.1f} fps)")

    for c in all_tracks.values():
        c["score"] = c["hits"] * np.log1p(c["npts"])
    sorted_t = sorted(all_tracks.values(), key=lambda t: -t["score"])

    print(f"\n  TOP-30 объектов:")
    for i, t in enumerate(sorted_t[:30]):
        print(f"  #{i+1:2d} dist={t['dist']:6.1f}м "
              f"hits={t['hits']:3d} zone={t['zone']:8s} "
              f"size=({t['sx']:5.2f},{t['sy']:5.2f},{t['sz']:5.2f})")

    zone_cnt = Counter(t["zone"] for t in all_tracks.values())
    max_d = max((t["dist"] for t in all_tracks.values()), default=0)
    print(f"\n  Всего уникальных: {len(all_tracks)}")
    print(f"  По зонам: {dict(zone_cnt)}")
    print(f"  Max dist: {max_d:.1f} м")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("usage: stream_zst_pipeline.py ZST_PATH OUT_CSV")
        sys.exit(1)
    main(sys.argv[1], sys.argv[2])