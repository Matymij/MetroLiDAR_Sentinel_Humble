# MetroLiDAR Sentinel — ROS 2 Humble

**Детекция препятствий в габарите поезда и на рельсовом пути** по данным LiDAR Pandar128E3X.

## Краткое описание

Пайплайн обрабатывает данные 128-канального LiDAR (307 200 точек/кадр), установленного на 1075 мм над рельсом по центру состава, и детектирует **любые объекты**, попадающие в габарит метропоезда (±1.35 м по ширине, 0.20–3.40 м по высоте от рельса).

**Производительность**: 100–156 fps (среднее ~120 fps) — **12× real-time** для 10 Гц лидара.  
**Максимальная дистанция детекции**: **208.8 м** (наблюдаемый максимум; instrumented range Pandar128E3X — 230 м, гарантированная детекция 2×2 м — до 200 м).

## Возможности

- **6 зон классификации**: `INSIDE` (в габарите), `NEAR` (граница), `RAIL` (на рельсе), `ABOVE` (свисающие), `OUTSIDE` (сбоку), `BELOW` (под рельсом — отсеивается)
- Полные bag'и без downsampling
- Потоковая обработка `.zst`-архивов без распаковки на диск
- Temporal tracking (min_hits=15) для отсева шума
- Дистанционно-адаптивные фильтры
- Numba JIT — **в 15× быстрее scipy**

## Требования

- Docker Desktop 4.x
- Windows 10/11, Linux, macOS
- RAM 8 ГБ, рекомендуется 16 ГБ
- Диск: 50 ГБ для датасетов

## Быстрый старт

```bash
# 1. Клонировать
git clone https://github.com/Matymij/MetroLiDAR_Sentinel_Humble.git
cd MetroLiDAR_Sentinel_Humble

# 2. Положить bag'и в ./bags/

# 3. Задать путь
export BAG_DIR="$(pwd)/bags"

# 4. Собрать образ
docker compose build sentinel

# 5. Запустить контейнер
docker compose run --rm -d --name sentinel_main sentinel bash -lc \
  'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && sleep infinity'
sleep 4

# 6. Скопировать bag в /tmp (быстрее)
docker exec sentinel_main bash -c '
  rm -rf /tmp/bag && mkdir -p /tmp/bag
  cp /bags/<bag-name>/*.db3 /tmp/bag/
  cp /bags/<bag-name>/metadata.yaml /tmp/bag/
'

# 7. Запустить пайплайн
docker exec -it sentinel_main bash -c '
  source /opt/ros/humble/setup.bash
  source /ws/install/setup.bash
  python3 -u /ws/scripts/offline_pipeline_v21_FINAL.py /tmp/bag /tmp/out.csv 6
'

# 8. Забрать результат
docker cp sentinel_main:/tmp/out.csv ./results/out.csv

src/                          # ROS2 пакеты
launch/                       # launch-файлы
config/                       # params.yaml, fastdds.xml
scripts/                      # Пайплайны
  offline_pipeline_v21_FINAL.py   # ⭐ Основной пайплайн
  stream_zst_pipeline.py      # Потоковая .zst
  consolidate_csv.py          # Пост-фильтр
  diagnose.py                 # Диагностика bag
docs/                         # Документация
results_v8/, results_v9/      # Результаты
FINAL_REPORT.md               # Итоговый отчёт

## Результаты

| Источник | Кадров | Объектов | Max dist |
|---|---|---|---|
| `cloud_with_fake_obj` | 1 510 | 70 | 190.5 м |
| `new_data` | 11 271 | 3 625 | 205.4 м |
| `for_hackathon` (6 bag'ов) | 2 488 | 190 | 208.8 м |
| **ВСЕГО** | **15 269** | **3 880** | **208.8 м** |

## Ограничения (Pandar128E3X)

Detection range: 200 м @ 10% reflectivity

Angular resolution: 0.1° H × 0.125° V

Физические пределы детекции:

Размер	Max
0.3×0.3 м	~60 м
0.5×0.5 м	~110 м
1×1 м	~160 м
2×2 м	190 м ✅
Невозможно: 500 м и 1000 м — данные за пределами instrumented range (230 м) отсутствуют.

## Технологии
ROS 2 Humble — middleware

numba JIT — компиляция Python в нативный код (×15)

numpy — массивы

ThreadPoolExecutor — параллелизм

Docker — воспроизводимость
