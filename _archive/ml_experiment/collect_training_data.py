"""Extract unique tracks from 6 bag CSVs for pseudo-labeling."""
import csv, ast, os

OUT = "results/v21_FINAL/training_data.csv"
HACK_DIR = "results/v21_FINAL/hackathon"

unique = {}
for fname in os.listdir(HACK_DIR):
    if not fname.endswith(".csv"):
        continue
    bag = fname.replace(".csv", "")
    with open(f"{HACK_DIR}/{fname}") as f:
        for r in csv.DictReader(f):
            try:
                dists = ast.literal_eval(r["dist_list"])
                zones = ast.literal_eval(r["zone_list"])
                sizes = ast.literal_eval(r["size_list"])
                tids = ast.literal_eval(r["track_id_list"])
            except Exception:
                continue
            for d, z, s, t in zip(dists, zones, sizes, tids):
                key = (bag, t)
                if key not in unique:
                    unique[key] = {
                        "bag": bag, "track_id": t,
                        "dist": d, "zone": z,
                        "sx": s[0], "sy": s[1], "sz": s[2],
                    }

print(f"Unique tracks: {len(unique)}")

os.makedirs("results/v21_FINAL", exist_ok=True)
with open(OUT, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=["bag", "track_id", "dist", "zone",
                                       "sx", "sy", "sz", "label"])
    w.writeheader()
    for r in unique.values():
        w.writerow({**r, "label": ""})

print(f"Saved: {OUT}")
