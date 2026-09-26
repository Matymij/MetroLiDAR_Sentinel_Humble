#!/usr/bin/env python3
"""Тест: обработать только первые 2 .db3 из zst."""
import sys, os, sqlite3, tarfile, shutil, time
import numpy as np
import zstandard

sys.path.insert(0, "/ws/scripts")
from stream_zst_pipeline import read_frames_from_db3
from offline_pipeline import _process_frame_impl

zst = sys.argv[1] if len(sys.argv) > 1 else "/bags/bags/new_data.zst"
tmp = "/tmp/test_stream.db3"

count = 0
with open(zst, "rb") as fz:
    d = zstandard.ZstdDecompressor()
    with d.stream_reader(fz) as r:
        with tarfile.open(fileobj=r, mode="r|") as tar:
            for m in tar:
                if not m.isfile() or not m.name.endswith(".db3"):
                    continue
                count += 1
                print(f"\n[{count}] {m.name} ({m.size/1e6:.0f} MB)")
                
                t0 = time.time()
                ex = tar.extractfile(m)
                with open(tmp, "wb") as o:
                    shutil.copyfileobj(ex, o, length=1<<20)
                print(f"  extracted in {time.time()-t0:.1f}s")
                
                t0 = time.time()
                n_frames = 0
                for ts, pts in read_frames_from_db3(tmp):
                    res = _process_frame_impl(ts, pts)
                    n_frames += 1
                    if n_frames <= 3:
                        print(f"    frame {n_frames}: in={pts.shape[0]} "
                              f"roi={res.get('roi_n',0)} "
                              f"objs={len(res.get('objects',[]))}")
                print(f"  {n_frames} frames in {time.time()-t0:.1f}s")
                
                os.remove(tmp)
                if count >= 2:
                    break

print(f"\nOK: обработано {count} файлов")