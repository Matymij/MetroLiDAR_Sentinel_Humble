"""Train LightGBM classifier on pseudo-labeled data."""
import csv, os
import numpy as np
import lightgbm as lgb
from sklearn.model_selection import train_test_split
from sklearn.metrics import precision_score, recall_score, f1_score

IN = "results/v21_FINAL/training_data_labeled.csv"
MODEL_DIR = "models"
os.makedirs(MODEL_DIR, exist_ok=True)

# Feature engineering
def features_from_row(r):
    sx, sy, sz = float(r["sx"]), float(r["sy"]), float(r["sz"])
    dims = sorted([sx, sy, sz])
    mind, midd, maxd = dims[0], dims[1], dims[2]
    aspect = maxd / max(mind, 1e-6)
    vol = sx * sy * sz
    dist = float(r["dist"])
    zone_id = {"INSIDE": 0, "NEAR": 1, "RAIL": 2, "ABOVE": 3,
               "OUTSIDE": 4, "BELOW": 5}.get(r["zone"], 6)
    return [
        sx, sy, sz, mind, midd, maxd,
        aspect, vol, dist, zone_id,
        mind / max(dist, 1.0),   # relative thinness
        maxd / max(dist, 1.0),   # relative size
    ]

FEATURE_NAMES = [
    "sx", "sy", "sz", "min_dim", "mid_dim", "max_dim",
    "aspect", "volume", "dist", "zone_id",
    "thinness_per_m", "size_per_m",
]

X, y = [], []
with open(IN) as f:
    for r in csv.DictReader(f):
        X.append(features_from_row(r))
        y.append(int(r["label"]))

X = np.array(X, dtype=np.float32)
y = np.array(y, dtype=np.int32)
print(f"Dataset: {X.shape}, positives: {y.sum()}")

X_tr, X_te, y_tr, y_te = train_test_split(X, y, test_size=0.2, random_state=42)

train_data = lgb.Dataset(X_tr, label=y_tr, feature_name=FEATURE_NAMES)
valid_data = lgb.Dataset(X_te, label=y_te, feature_name=FEATURE_NAMES)

params = {
    "objective": "binary",
    "metric": "binary_logloss",
    "learning_rate": 0.1,
    "num_leaves": 15,
    "min_data_in_leaf": 5,
    "verbose": -1,
}

booster = lgb.train(
    params, train_data,
    num_boost_round=100,
    valid_sets=[valid_data],
    callbacks=[lgb.early_stopping(20), lgb.log_evaluation(20)],
)

# Evaluation
y_pred = (booster.predict(X_te) > 0.5).astype(int)
print(f"\nValidation:")
print(f"  Precision: {precision_score(y_te, y_pred, zero_division=0):.3f}")
print(f"  Recall:    {recall_score(y_te, y_pred, zero_division=0):.3f}")
print(f"  F1:        {f1_score(y_te, y_pred, zero_division=0):.3f}")

booster.save_model(f"{MODEL_DIR}/classifier.txt")
print(f"\nModel saved: {MODEL_DIR}/classifier.txt")

# Feature importance
print("\nTop features:")
for name, imp in sorted(zip(FEATURE_NAMES, booster.feature_importance()),
                        key=lambda x: -x[1])[:6]:
    print(f"  {name:20s} {imp}")
