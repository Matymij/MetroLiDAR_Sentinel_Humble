#!/usr/bin/env python3
"""Глобальная консолидация: слияние объектов по (zone, dist±3м, size±30%)."""
import csv
import sys
from collections import defaultdict

import numpy as np


def main(in_csv, out_csv):
    all_objects = []
    with open(in_csv) as f:
        r = csv.DictReader(f)
        for row in r:
            dists = eval(row["dist_list"]) if row["dist_list"] else []
            zones = eval(row["zone_list"]) if row["zone_list"] else []
            sizes = eval(row["size_list"]) if row["size_list"] else []
            for i in range(min(len(dists), len(zones), len(sizes))):
                all_objects.append({
                    "file": row["file"],
                    "dist": dists[i],
                    "zone": zones[i],
                    "size": sizes[i],
                })

    print(f"raw: {len(all_objects)}")

    # Bucket по zone и dist/3
    buckets = defaultdict(list)
    for obj in all_objects:
        key = (obj["zone"], round(obj["dist"] / 3.0))
        buckets[key].append(obj)

    # Внутри бакета — кластер по позиции+размеру
    merged_objects = []
    for key, group in buckets.items():
        # Сортируем по dist
        group.sort(key=lambda o: o["dist"])
        used = [False] * len(group)
        for i, o in enumerate(group):
            if used[i]:
                continue
            cluster = [o]
            used[i] = True
            for j in range(i + 1, len(group)):
                if used[j]:
                    continue
                o2 = group[j]
                if abs(o["dist"] - o2["dist"]) > 3.0:
                    continue
                # size совместим?
                s1 = np.array(o["size"])
                s2 = np.array(o2["size"])
                size_diff = np.abs(s1 - s2) / (np.maximum(s1, s2) + 1e-6)
                if size_diff.max() < 0.5:
                    cluster.append(o2)
                    used[j] = True
            merged_objects.append(cluster)

    print(f"после слияния: {len(merged_objects)}")

    # Сохраняем
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["zone", "dist_m", "size_x", "size_y", "size_z",
                    "n_sources", "n_files"])
        for cluster in merged_objects:
            dists = [c["dist"] for c in cluster]
            sizes = np.array([c["size"] for c in cluster])
            files = set(c["file"] for c in cluster)
            w.writerow([
                cluster[0]["zone"],
                round(float(np.median(dists)), 1),
                round(float(np.median(sizes[:, 0])), 2),
                round(float(np.median(sizes[:, 1])), 2),
                round(float(np.median(sizes[:, 2])), 2),
                len(cluster),
                len(files),
            ])

    # Итоговая статистика
    zones = defaultdict(int)
    dists_all = []
    for cluster in merged_objects:
        zones[cluster[0]["zone"]] += 1
        dists_all.append(np.median([c["dist"] for c in cluster]))

    print(f"\n=== ИТОГ ===")
    print(f"Уникальных объектов: {len(merged_objects)}")
    print(f"Зоны: {dict(zones)}")
    if dists_all:
        print(f"Max dist: {max(dists_all):.1f} м")
        print(f"Средняя dist: {np.mean(dists_all):.1f} м")

    print(f"\nРаспределение:")
    bins = [(0, 20), (20, 50), (50, 100), (100, 150), (150, 210)]
    for lo, hi in bins:
        n = sum(1 for d in dists_all if lo <= d < hi)
        bar = "#" * min(n, 50)
        print(f"  {lo:3d}-{hi:3d}м: {n:4d} {bar}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])