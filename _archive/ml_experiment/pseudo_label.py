"""Generate pseudo-labels from plausibility rules + train LightGBM."""
import csv, ast, sys, os
import numpy as np

sys.path.insert(0, "scripts")
from plausibility import check_plausibility

IN = "results/v21_FINAL/training_data.csv"
OUT = "results/v21_FINAL/training_data_labeled.csv"

rows = []
with open(IN) as f:
    for r in csv.DictReader(f):
        d = float(r["dist"])
        sx, sy, sz = float(r["sx"]), float(r["sy"]), float(r["sz"])
        zone = r["zone"]

        # Reconstruct mn/mx from size (bbox shape doesn't matter for plausibility, only dims)
        mn = np.array([d - sx/2, -sy/2, 0.5])
        mx = np.array([d + sx/2,  sy/2, 0.5 + sz])

        # Pseudo-label via teacher rules
        ok, _ = check_plausibility(mn, mx, zone, d, npts=10)
        r["label"] = 1 if ok else 0
        rows.append(r)

n_pos = sum(1 for r in rows if r["label"] == "1")
n_neg = len(rows) - n_pos
print(f"Pseudo-labeled: {n_pos} positive, {n_neg} negative")

with open(OUT, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader()
    w.writerows(rows)
print(f"Saved: {OUT}")
