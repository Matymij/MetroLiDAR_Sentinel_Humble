"""ML classifier wrapper — with safe fallback to v21 rules."""
import os
import numpy as np

_MODEL_PATH = os.path.join(os.path.dirname(__file__), "..", "models", "classifier.txt")
_booster = None
_HAS_ML = False

try:
    if os.path.exists(_MODEL_PATH):
        import lightgbm as lgb
        _booster = lgb.Booster(model_file=_MODEL_PATH)
        _HAS_ML = True
        print(f"[ml_classifier] Model loaded: {_MODEL_PATH}", flush=True)
    else:
        print(f"[ml_classifier] No model at {_MODEL_PATH}, using rules only", flush=True)
except Exception as e:
    print(f"[ml_classifier] Load failed: {e}, using rules only", flush=True)


def ml_score(zone, dist, sx, sy, sz):
    """Return 0..1 score. 1.0 if ML unavailable (fallback to accept)."""
    if not _HAS_ML:
        return 1.0
    dims = sorted([sx, sy, sz])
    mind, midd, maxd = dims[0], dims[1], dims[2]
    aspect = maxd / max(mind, 1e-6)
    vol = sx * sy * sz
    zone_id = {"INSIDE": 0, "NEAR": 1, "RAIL": 2, "ABOVE": 3,
               "OUTSIDE": 4, "BELOW": 5}.get(zone, 6)
    f = np.array([[
        sx, sy, sz, mind, midd, maxd,
        aspect, vol, dist, zone_id,
        mind / max(dist, 1.0),
        maxd / max(dist, 1.0),
    ]], dtype=np.float32)
    return float(_booster.predict(f)[0])
