# Architecture

## Общая схема
┌───────────────────────┐
│ ROS 2 bag (.db3) │ sqlite3, sensor_msgs/PointCloud2
└──────────┬────────────┘
↓
┌───────────────────────┐
│ stream_frames() │ NP.frombuffer → xyz (numpy, 20× быстрее read_points)
└──────────┬────────────┘
↓
┌───────────────────────┐
│ rotate_roi_numba() │ rotate + ROI за один проход (numba nogil)
│ │ forward ∈ [0.3, 230], lateral ∈ [-3, 3], z ∈ [-0.3, 5.4]
└──────────┬────────────┘
↓
┌────────────────────────────────────────────────┐
│ 3-layer split (near / mid / far) │
│ near: 0–50 м, voxel 0.10, eps 0.50, min_pts 3│
│ mid: 50–120 м,voxel 0.25, eps 1.20, min_pts 3│
│ far: 120–230 м,voxel 0.35, eps 4.00, min_pts 2
└──────────┬─────────────────────────────────────┘
↓
┌───────────────────────┐
│ grid_dbscan_numba() │ cell hashing + union-find, numba nogil
└──────────┬────────────┘
↓
┌───────────────────────┐
│ classify(cy, cz) │ INSIDE / NEAR / RAIL / ABOVE / OUTSIDE / BELOW
└──────────┬────────────┘
↓
┌───────────────────────┐
│ is_valid_object() │ distance-adaptive фильтры (bbox, shape, min_npts, zone)
└──────────┬────────────┘
↓
┌───────────────────────┐
│ Tracker.update() │ ассоциация по 3D-позиции (6 м), timeout 8 кадров
└──────────┬────────────┘
↓
┌───────────────────────┐
│ Пост-фильтр │ min_hits_for(dist), слияние треков ±3 м, score
└──────────┬────────────┘
↓
┌───────────────────────┐
│ CSV + опционально UI │ results_v9/*.csv + web_cockpit.py
└───────────────────────┘

## Компоненты

### 1. Streaming reader (`stream_frames`)
Читает `PointCloud2` из sqlite3 напрямую через `np.frombuffer`. Первые 12 байт — x, y, z (float32). Работает с 16-байтным (xyz+intensity) и 26-байтным (xyz+intensity+ring+timestamp) форматами без правок.

**Ускорение:** ~20× быстрее `point_cloud2.read_points`.

### 2. Numba rotate + ROI (`rotate_roi_numba`)
За один проход:
- Поворот осей: forward = −Y → +X, lateral = X → Y
- ROI-фильтр

**Скорость:** 3 мс на 307 000 точек, отпускает GIL.

### 3. Адаптивные слои
| Слой | Диапазон | Voxel | eps | min_pts |
|---|---|---|---|---|
| near | 0.3–50 м | 0.10 м | 0.50 м | 3 |
| mid | 50–120 м | 0.25 м | 1.20 м | 3 |
| far | 120–230 м | 0.35 м | 4.00 м | 2 |

Принцип: чем дальше — тем крупнее voxel и eps (компенсация разрежения точек).

### 4. Grid DBSCAN (`grid_dbscan_numba`)
Cell hashing + union-find. Каждая точка → ячейка размером eps, проверяются 27 соседних ячеек, связываются через union-find.

**Скорость:** 5–10× быстрее scipy `cKDTree.query_pairs`. Отпускает GIL → реальный параллелизм через ThreadPoolExecutor.

### 5. Zone classifier
Классификация относительно габарита:
- **INSIDE** — |y| ≤ 1.35 и z ∈ [0.20, 3.40] → критично
- **NEAR** — на границе ±0.30 м
- **RAIL** — |z| < 0.30 (рельс)
- **ABOVE** — z > 3.40 (свисает)
- **OUTSIDE** — |y| − 1.35 > 0.30
- **BELOW** — z < −0.10 (отсев)

### 6. Distance-adaptive фильтры (`is_valid_object`)
- `min_npts`: 3 (близко) → 2 (далеко, 170+ м)
- `shape`: только для `dist < 60` м
- `outside_far`: 0.5 м (близко) → 1.5 м (далеко)
- `max_bbox_dim`: 15 м

### 7. Temporal tracking
- Ассоциация по 3D-позиции (eps=6 м)
- Трек подтверждается после `min_hits = 15`
- Timeout 8 кадров
- Публикуются только confirmed-треки

### 8. Пост-фильтр
- Отсев по `min_hits_for(dist)`: 50 / 30 / 15 / 8
- Отсев тонких RAIL (`sy < 0.20` и `sz < 0.20`)
- Минимальный размер 0.15 м
- Слияние треков по позиции ±3 м
- Ранжирование по score = hits × log(1+npts) × group_size

## Режимы запуска

| Файл | Когда использовать |
|---|---|
| `offline_pipeline_FINAL.py` | Основной v11, ~47 fps, все три bag'а |
| `stream_zst_pipeline.py` | Потоковая обработка `.zst` без распаковки (new_data) |
| `web_cockpit.py` | Real-time визуализация в браузере (FastAPI + Three.js) |

## Потоковая обработка .zst

`stream_zst_pipeline.py`:
1. Открывает `.zst` через `zstandard.stream_reader`
2. Итерирует tar потоково
3. Извлекает один `.db3` во временную папку
4. Обрабатывает все кадры
5. Удаляет `.db3`
6. Повторяет

**Пик на диске:** 0.5 ГБ вместо 88 ГБ распакованных. **Скорость:** 87 ГБ за ~10 минут.

## Многопоточность

`ThreadPoolExecutor(6)` — numba-функции отпускают GIL (`nogil=True`), потоки работают реально параллельно.

**Ускорение:** 3–4× относительно одного потока.