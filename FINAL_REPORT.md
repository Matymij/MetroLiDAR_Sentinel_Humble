# MetroLiDAR Sentinel — Финальный отчёт

## 1. Задача
Детекция любых объектов, попадающих в габарит поезда метро или на рельсовый путь, по данным LiDAR Pandar128E3X, установленного на 1075 мм над рельсом по центру состава.

## 2. Аппаратура
- **LiDAR:** Pandar128E3X (128 каналов, 0.1° горизонт / 0.125° вертикаль)
- **Instrumented range:** 230 м
- **Detection range:** 200 м @ 10% reflectivity
- **Позиция:** 1075 мм над рельсом, по центру

## 3. Габарит
- Ширина: ±1.35 м от центра (y)
- Высота от рельса: 0.20–3.40 м (z)
- Рельс: z = 0 в системе лидара
- Forward: −Y (после rotate → +X)

## 4. Архитектура pipeline

ROS2 bag (.db3) → [streaming reader]
↓
[3-layer preprocess по дистанции]
near: 0–50 м voxel=4см, eps=0.5
mid: 50–120 м voxel=15см, eps=1.2
far: 120–230 м voxel=25см, eps=5.0
↓
[Fast DBSCAN] (cKDTree + union-find)
↓
[Zone classification]
INSIDE / NEAR / RAIL / ABOVE / OUTSIDE / BELOW
↓
[Distance-adaptive фильтры]
↓
[Temporal tracking]
min_hits ≥ 15 (адаптивно по дистанции)
↓
[Пост-фильтр: кластеризация по позиции ±3 м]
↓
CSV со всеми объектами


## 5. Результаты

### Общая сводка
| Источник | Кадров | Объектов | Max dist |
|---|---|---|---|
| `cloud_with_fake_obj` | 1 510 | 65 | 190.5 м |
| `new_data` | 11 271 | 3 625 | 205.4 м |
| `for_hackathon` (6 bag'ов) | 2 488 | 190 | 208.8 м |
| **ВСЕГО** | **15 269** | **3 880** | **208.8 м** |

### По сценариям for_hackathon
| Сценарий | Кадры | Объектов | Max dist |
|---|---|---|---|
| doubleT_obstacle | 201 | 21 | 119.7 м |
| doubleT_platform | 345 | 33 | 193.8 м |
| roundT_doubleT | 252 | 23 | 118.9 м |
| roundT_pressureGate_roundT | 268 | 12 | 119.6 м |
| roundT_squareT_pressureGate_squareT | 545 | 49 | 196.5 м |
| squareT_platform_squareT_switch | 877 | 52 | 208.8 м |

## 6. Физические ограничения (честно)

**Pandar128E3X физически не видит:**
- Дальше 230 м — instrumented range
- 0.3×0.3 м дальше ~60 м (угол 0.17° < шага луча)
- 1×1 м дальше ~130 м
- 2×2 м детектируется до ~190 м

**1000 м и даже 500 м — невозможны физически.** Максимум — 208.8 м (достигнут).

## 7. Зоны классификации
- **INSIDE** — внутри габарита (критично)
- **NEAR** — на границе ±0.3 м
- **RAIL** — на рельсовом пути
- **ABOVE** — сверху (свисающие)
- **OUTSIDE** — сбоку от габарита
- **BELOW** — под рельсом (шум/балласт — отсеивается)

## 8. Производительность
- Скорость: **36–55 fps** (среднее ~47, 4.7× real-time для 10 Гц LiDAR)
- CPU: 6 воркеров (ThreadPoolExecutor + numba nogil)
- RAM: 300 МБ – 1 ГБ
- Время на 11 271 кадр new_data: **10 минут**
- Время на 1 510 кадров cloud_with_fake_obj: **26 секунд**

## 9. Ограничения системы
- Стрелки метро могут давать ложные срабатывания (по ТЗ не штрафуется)
- Очень плотный шум от стен тоннеля требует строгих фильтров
- 0.3×0.3 м надёжно детектируется только до 50–60 м

## 10. Артефакты

**Код:**
- `scripts/offline_pipeline_FINAL.py` — основной пайплайн (v11, 36–55 fps)
- `scripts/stream_zst_pipeline.py` — потоковая обработка `.zst`
- `scripts/offline_pipeline.py` — v9 (для сравнения)
- `src/` — ROS 2 пакеты (C++ preprocessor, Python detector, msgs)

**Результаты (в репозитории):**
- `results_v9/v11_FINAL.csv` — прогон на `cloud_with_fake_obj` (1 510 кадров, 65 объектов, 190.5 м)
- `results_v9/new_data_FINAL.csv` — прогон на `new_data` (11 271 кадр, 3 625 объектов, 205.4 м)
- `results_v9/doubleT_obstacle.csv`, `doubleT_platform.csv`, `roundT_doubleT.csv`, `roundT_pressureGate_roundT.csv`, `roundT_squareT_pressureGate_squareT.csv`, `squareT_platform_squareT_switch.csv` — 6 bag'ов for_hackathon (2 488 кадров, 190 объектов, 208.8 м)
- `results/final/final_ok.csv` — финальная сводка
- `results_v9/*.log` — логи прогонов

**Документация:**
- `README.md`, `README_RU.md` — инструкция запуска
- `docs/ARCHITECTURE.md`, `docs/ALGORITHM.md`, `docs/BENCHMARK.md`
- `FINAL_REPORT.md` — этот файл