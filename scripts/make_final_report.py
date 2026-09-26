#!/usr/bin/env python3
"""Финальный CSV со сводкой по всем прогонам."""
import csv, os, sys
from collections import Counter

OUT = "results_v9/SUMMARY.csv"

rows = []
# for_hackathon v9
for name in ["doubleT_obstacle", "doubleT_platform", "roundT_doubleT",
             "roundT_pressureGate_roundT", "roundT_squareT_pressureGate_squareT",
             "squareT_platform_squareT_switch"]:
    csvf = f"results_v9/{name}.csv"
    logf = f"results_v9/{name}.log"
    if not os.path.exists(logf):
        continue
    frames = 0
    maxd = 0.0
    nobj = 0
    zones = Counter()
    with open(logf) as f:
        for line in f:
            if "Всего уникальных" in line:
                nobj = int(line.split(":")[-1].strip())
            if "Max dist:" in line:
                maxd = float(line.split(":")[1].split("м")[0].strip())
            if line.startswith("doubleT") or line.startswith("roundT") or line.startswith("squareT"):
                pass
    with open(csvf) as f:
        frames = sum(1 for _ in f) - 1
    rows.append(("for_hackathon", name, frames, nobj, maxd))

# new_data
with open("results_v8/new_data_consolidated.csv") as f:
    r = csv.DictReader(f)
    n = sum(1 for _ in r)
    r = csv.DictReader(open("results_v8/new_data_consolidated.csv"))
    dists = [float(row["dist_m"]) for row in r]
    maxd = max(dists) if dists else 0
rows.append(("new_data", "stream", 11271, n, maxd))

# cloud_with_fake_obj
rows.append(("cloud_with_fake_obj", "v9", 1510, 75, 199.8))

with open(OUT, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["source", "bag", "frames", "objects", "max_dist_m"])
    for r in rows:
        w.writerow(r)

print(f"OK: {OUT}")
for r in rows:
    print(f"  {r[0]:20s} {r[1]:40s} {r[2]:6d} кадров, {r[3]:5d} объектов, max={r[4]:.1f} м")
