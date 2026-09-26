#!/usr/bin/env python3
"""Диагностика bag: структура PointCloud2, распределение координат, оси."""
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def main(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    print(f"topics: {topics}\n")

    cloud_topic = next((n for n, t in topics.items()
                        if t == "sensor_msgs/msg/PointCloud2"), None)
    if cloud_topic is None:
        print("no PointCloud2")
        return

    frames = []
    while reader.has_next() and len(frames) < 3:
        topic, data, ts = reader.read_next()
        if topic != cloud_topic:
            continue
        msg = deserialize_message(data, PointCloud2)
        frames.append(msg)

    if not frames:
        print("no frames")
        return

    msg = frames[0]
    print(f"=== PointCloud2 структура ===")
    print(f"  height={msg.height} width={msg.width} "
          f"point_step={msg.point_step} row_step={msg.row_step}")
    print(f"  is_dense={msg.is_dense}")
    print(f"  fields:")
    for f in msg.fields:
        print(f"    {f.name:12s}  offset={f.offset:3d}  "
              f"datatype={f.datatype}  count={f.count}")
    print(f"  msg.header.frame_id = {msg.header.frame_id}")

    print(f"\n=== XYZ распределение (3 кадра) ===")
    all_x, all_y, all_z = [], [], []
    all_int = []
    has_intensity = any(f.name == "intensity" for f in msg.fields)

    for i, m in enumerate(frames):
        arr = point_cloud2.read_points(
            m, field_names=("x", "y", "z"), skip_nans=True)
        x = arr['x'].astype(np.float32)
        y = arr['y'].astype(np.float32)
        z = arr['z'].astype(np.float32)
        all_x.append(x); all_y.append(y); all_z.append(z)

        if has_intensity:
            try:
                arr_i = point_cloud2.read_points(
                    m, field_names=("x", "y", "z", "intensity"), skip_nans=True)
                all_int.append(arr_i['intensity'].astype(np.float32))
            except Exception:
                pass

        print(f"\n--- frame {i}: {len(x)} точек ---")
        print(f"  x: {x.min():8.2f}  ..  {x.max():8.2f}   "
              f"mean={x.mean():7.2f}  p50={np.percentile(x,50):7.2f}")
        print(f"  y: {y.min():8.2f}  ..  {y.max():8.2f}   "
              f"mean={y.mean():7.2f}  p50={np.percentile(y,50):7.2f}")
        print(f"  z: {z.min():8.2f}  ..  {z.max():8.2f}   "
              f"mean={z.mean():7.2f}  p50={np.percentile(z,50):7.2f}")

        # Гистограмма по z — понять где пол
        zh, z_edges = np.histogram(z, bins=20)
        print(f"  Z-гистограмма (пол должен быть пик):")
        for i_h, c in enumerate(zh):
            bar = "#" * int(40 * c / max(zh.max(), 1))
            print(f"    {z_edges[i_h]:6.2f}..{z_edges[i_h+1]:6.2f}: {bar} {c}")

    print(f"\n=== Итог по 3 кадрам ===")
    X = np.concatenate(all_x); Y = np.concatenate(all_y); Z = np.concatenate(all_z)
    print(f"  N = {len(X)}")
    print(f"  x range: {X.min():.2f} .. {X.max():.2f}  (span={X.max()-X.min():.2f})")
    print(f"  y range: {Y.min():.2f} .. {Y.max():.2f}  (span={Y.max()-Y.min():.2f})")
    print(f"  z range: {Z.min():.2f} .. {Z.max():.2f}  (span={Z.max()-Z.min():.2f})")

    # Какая ось "forward" — та, что имеет максимальный span
    spans = {"x": X.max()-X.min(), "y": Y.max()-Y.min(), "z": Z.max()-Z.min()}
    print(f"  spans: {spans}")
    print(f"  → forward axis скорее всего = {max(spans, key=spans.get).upper()}")
    print(f"  → up axis скорее всего = Z (пол внизу, потолок сверху)")

    if all_int:
        I = np.concatenate(all_int)
        print(f"  intensity: {I.min():.2f} .. {I.max():.2f}  mean={I.mean():.2f}")

    # Гистограмма y (боковое) — где стены
    yh, y_e = np.histogram(Y, bins=20)
    print(f"\n  Y-гистограмма (стены — пики по краям):")
    for i_h, c in enumerate(yh):
        bar = "#" * int(40 * c / max(yh.max(), 1))
        print(f"    {y_e[i_h]:7.2f}..{y_e[i_h+1]:7.2f}: {bar} {c}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "/bags/for_hackathon/doubleT_platform")