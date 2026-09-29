# MetroLiDAR Sentinel — Real-time LiDAR obstacle detection

**Real-time 3D-LiDAR obstacle detection for a metro train envelope.**
Detects any object inside the train gauge (±1.35 m width, 0.20–3.40 m above rail) or on the track, using a Pandar128E3X LiDAR mounted at 1075 mm above rail, centered on the train.

**Русская версия:** [README_RU.md](README_RU.md)

## Key metrics

| Metric | Value |
|---|---|
| Frames processed | 3 998 (1510 + 2488) |
| Unique objects detected | 86 DANGEROUS |
| Throughput | **63–86 fps** (Windows) / **156 fps** (Linux), ~6–16× real-time for 10 Hz LiDAR |
| Max detection range | **188.2 m** (TOP-12 confirmed; raw tracker max 203.7 m; sensor limit 208 m) |
| Gauge compliance zones | 6 (`INSIDE`, `NEAR`, `RAIL`, `ABOVE`, `OUTSIDE`, `BELOW`) |

## Requirements

- Python 3.10+
- pip install numpy scipy numba rosbags

No Docker or ROS2 required.
- Windows 10/11, Linux, or macOS
- RAM: 8 GB min, 16 GB recommended
- Disk: 50 GB for datasets (bags are **not** in the repo)

### Windows: important notes

If you run on Windows with Docker Desktop + Git Bash, three issues may bite:

1. **Line endings in entrypoint.sh.** Fixed via `.gitattributes` (`*.sh text eol=lf`). If your clone was made before this file, run:
   ```bash
   git rm --cached -r . && git reset --hard
   ```
2. **MSYS path conversion.** Git Bash rewrites `/out` → `C:/Program Files/Git/out`. Always prefix `docker run -v` with `MSYS_NO_PATHCONV=1`:
   ```bash
   MSYS_NO_PATHCONV=1 docker run --rm -v "C:/path/to/bags:/bags:ro" ...
   ```
3. **Non-ASCII or spaces in BAG_DIR.** Docker Desktop may not bind-mount Cyrillic paths. Copy bags to an ASCII path first:
   ```bash
   mkdir -p /c/VSC/bags_ascii/cloud_with_fake_obj
   cp "/c/VSC/Проект №5/.../cloud_with_fake_obj_0.db3" /c/VSC/bags_ascii/cloud_with_fake_obj/
   ```

## Quick start

### Option A: Python-only (no Docker, recommended)

Requires: Python 3.10+ and `pip install numpy scipy numba rosbags`.

```bash
# 1. Clone
git clone https://github.com/Matymij/MetroLiDAR_Sentinel_Humble.git
cd MetroLiDAR_Sentinel_Humble

# 2. Install dependencies
pip install numpy scipy numba rosbags

# 3. Place ROS 2 bags under ./bags/, then run:
python -u scripts/offline_pipeline_v23_FAST.py \
    bags/cloud_with_fake_obj results/out.csv 8
```

### Option B: Docker

```bash
# 1. Clone
git clone https://github.com/Matymij/MetroLiDAR_Sentinel_Humble.git
cd MetroLiDAR_Sentinel_Humble

# 2. Place ROS 2 bags under ./bags/
export BAG_DIR="$(pwd)/bags"

# 3. Build the image
docker compose build sentinel

# 4. Start the container in the background
docker compose run --rm -d --name sentinel_main sentinel bash -lc \
  'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && sleep infinity'
sleep 5

# 5. Run the offline pipeline
docker exec -it sentinel_main bash -c '
  source /opt/ros/humble/setup.bash
  source /ws/install/setup.bash
  python3 -u /ws/scripts/offline_pipeline_v23_FAST.py \
    /bags/cloud_with_fake_obj /tmp/out.csv 6
'

# 6. Collect the result
docker cp sentinel_main:/tmp/out.csv ./results/out.csv
```

Typical runtime on a modern CPU:

| Input | Frames | Time | DANGEROUS | Max dist |
|---|---|---|---|---|
| `cloud_with_fake_obj` | 1 510 | 17.6 s | 26 | 188.2 m |
| `for_hackathon` (6 bags) | 2 488 | 45.9 s | 60 | 158.9 m |
| **TOTAL** | **3 998** | **63.5 s** | **86** | **188.2 m** |

## Repository layout

```text
config/         params.yaml, fastdds.xml
docker/         entrypoint.sh
docs/           ARCHITECTURE.md, ALGORITHM.md, BENCHMARK.md
launch/         metro_sentinel.launch.py, algorithm_only.launch.py
results/        final/final_ok.csv
results_v9/     full CSV + logs for all three datasets
rviz/           metro_sentinel.rviz
scripts/        offline_pipeline_v23_FAST.py (main), stream_zst_pipeline.py
src/            ROS 2 packages (C++ preprocessor, Python detector, msgs)
Datasets
Bags are not committed (7+ GB each). The report uses three sources:

- `cloud_with_fake_obj` — synthetic obstacles in the gauge (ground truth).
- `for_hackathon` — 6 tunnel scenarios (doubleT, roundT, squareT, pressure gate, switch).
- `new_data` — 11 271 frames from 215+ tunnel recordings.

All results are in [`results_v9/`](results_v9/) as CSV + logs.

## Documentation

- [FINAL_REPORT.md](FINAL_REPORT.md) — full project report (RU)
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) — pipeline diagram
- [docs/ALGORITHM.md](docs/ALGORITHM.md) — detection algorithm
- [docs/BENCHMARK.md](docs/BENCHMARK.md) — performance benchmarks
- [README_RU.md](README_RU.md) — full Russian guide
---

## v23 FINAL — Current pipeline

Main pipeline: `scripts/offline_pipeline_v23_FAST.py`

### Improvements over v11

1. **Bbox-gauge intersection** — classifies by bounding box × envelope,
   not just center. Distance-adaptive margins (5/10/15 cm).
2. **Rail auto-detection** — objects touching rail = DANGEROUS regardless of height.
3. **Two-list output**:
   - DANGEROUS (top-12): in-gauge objects
   - WATCHLIST (top-6): outside-gauge objects
4. **Temporal gating** — near objects need 3/5 recent-frame visibility

### New modules

- `path_analysis.py` — rail curvature detection (STRAIGHT/GENTLE/SHARP)
- `plausibility.py` — rejects impossible detections (walls, ceiling, ghost)
- `confidence.py` — 0..1 score (track × size × zone × distance)

### Metrics (cloud_with_fake_obj, 1510 frames)

| Metric | v11 | v23 |
|---|---|---|
| DANGEROUS | 0 | 28 |
| TOP-12 max dist | 190.5 m | 188.2 m |
| Rail contacts | 0 | 2 |
| FPS Windows | 89 | 84 |
| FPS Linux | 156 | 156 |

### Repository structure

- `scripts/` — production pipeline only
- `results/v21_FINAL/` — fresh benchmarks
- `_archive/` — version history (v6–v20)
