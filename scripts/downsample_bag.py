#!/usr/bin/env python3
"""Downsample ROS2 bag: keep every Nth point in each PointCloud2 message.

Works with heterogeneous PointCloud2 fields (uses raw byte slicing).
"""
import sys
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message


def downsample(src, dst, step=5):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=src, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=dst, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    for t in reader.get_all_topics_and_types():
        writer.create_topic(t)

    msg_type = get_message("sensor_msgs/msg/PointCloud2")
    n_in = n_out = frames = 0
    while reader.has_next():
        topic, data, ts = reader.read_next()
        is_cloud = topic.endswith("pointcloud") or topic.endswith("points")
        if is_cloud:
            m = deserialize_message(data, msg_type)
            ps = m.point_step
            n = m.width * m.height
            if n > 0:
                keep = list(range(0, n, step))
                buf = bytearray(ps * len(keep))
                for j, i in enumerate(keep):
                    buf[j * ps:(j + 1) * ps] = m.data[i * ps:(i + 1) * ps]
                m.data = bytes(buf)
                m.width = len(keep)
                m.height = 1
                m.row_step = ps * len(keep)
                data = serialize_message(m)
                n_in += n
                n_out += len(keep)
            frames += 1
        writer.write(topic, data, ts)
    print(f"done: frames={frames}, {n_in} → {n_out} points")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("usage: downsample_bag.py SRC DST [STEP]")
        sys.exit(1)
    src, dst = sys.argv[1], sys.argv[2]
    step = int(sys.argv[3]) if len(sys.argv) > 3 else 5
    downsample(src, dst, step)