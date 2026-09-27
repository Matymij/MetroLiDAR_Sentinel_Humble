# Архив версий pipeline

История разработки пайплайна детекции препятствий. Основной рабочий файл — **`scripts/offline_pipeline_v11_FINAL.py`**.

## Что в `pipeline_versions/`

| Версия | Особенность | fps (cloud_with_fake_obj) |
|---|---|---|
| `offline_pipeline.py` | Оригинал (v5) | 30–45 |
| `offline_pipeline_FINAL.py` | v6 — эталон до оптимизации | 50 |
| `offline_pipeline_OPT.py` | v7 — CPU baseline (cKDTree) | 70 |
| `offline_pipeline_v9.py` | Эксперимент с грубым voxel 0.20 | 172 (потеря точности) |
| `offline_pipeline_v9a.py` | Адаптивный voxel по дистанции | 82 (много артефактов) |
| `offline_pipeline_v9b.py` | Усиленные фильтры, но min_pts=5 жёсткий | 59 |
| `offline_pipeline_v10.py` | Voxel 0.15 для near-слоя — **прорыв** | 127 |
| `offline_pipeline_v11_STABLE.py` | Промежуточная | 100 |
| **`offline_pipeline_v11_FINAL.py`** | **Текущий чемпион (в scripts/)** | **156** |
| `offline_pipeline_v12.py` | Усиленный фильтр плоских артефактов | 128 |
| `offline_pipeline_v13_PREFETCH.py` | Reader thread + очередь — не сработал | 112 |
| `offline_pipeline_GPU.py` | RAPIDS cuML на CUDA — проиграл | 44 |
| `profile_gpu.py` | Профилировщик слоёв | — |
| `activate_gpu.sh` | WSL helper для RAPIDS (архив) | — |
| `web_cockpit_v6.py.bak` | UI на v6 (до переключения на v11) | — |

## Почему GPU в архиве

Проверена выгрузка DBSCAN на GPU (RAPIDS cuML 24.12, RTX 4070 Laptop, 8 GB):

| Метрика | CPU v11 | GPU cuML |
|---|---|---|
| Скорость | **128–156 fps** | 44 fps |
| Объектов на эталоне | **70** | 68 |
| Требования | numpy + numba | + cudf + cuml + cugraph (~2.5 GB) |

**Причина:** на кадрах <50k точек/слой overhead переноса CPU↔GPU (~200 µs на вызов) не окупается. GPU выигрывает при 50k+ точек **на один вызов** — это требует batched cuML (8–16 кадров за раз, отдельный проект на 2–4 недели).

## Что сработало (v11_FINAL)

1. Voxel downsampling **0.10 → 0.15** только для near-слоя — главный ускоритель (×1.8)
2. **8 воркеров** вместо 4
3. **cKDTree** вместо numba grid_dbscan для облаков >800 точек
4. Замена **`rosbag2_py` на `rosbags`** — работает без ROS2 и Docker
5. Усиленный фильтр плоских артефактов (сторона <5 см на дистанции >60 м)

См. `FINAL_REPORT.md` §11 для подробностей и полной таблицы бенчмарка.

## Старые вспомогательные скрипты

В `scripts_old/` лежат устаревшие runner-скрипты эпохи Docker+ROS2:

- `run_all.sh`, `run_bag.sh`, `run_pipeline.sh` — обёртки `docker exec` вокруг пайплайна
- `run_all_v9.sh` — то же для v9
- `ros2x.sh` — helper для ROS2-окружения
- `profile_cpu_v9.py` — CPU-профилировщик на v9

Все они **не нужны** в текущей архитектуре (работаем без Docker и ROS2 через `rosbags` + venv).
