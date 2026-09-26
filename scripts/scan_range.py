#!/usr/bin/env python3
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class Scan(Node):
    def __init__(self, topic):
        super().__init__("scan")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.got = False
        self.create_subscription(PointCloud2, topic, self.cb, qos)
        print(f"subscribed to {topic} (qos=RELIABLE/VOLATILE/depth=10)", flush=True)

    def cb(self, msg):
        arr = point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        x = arr["x"].astype(np.float32)
        y = arr["y"].astype(np.float32)
        z = arr["z"].astype(np.float32)
        r = np.sqrt(x * x + y * y + z * z)

        print(f"points: {len(r)}")
        print(f"  x: {x.min():.2f} .. {x.max():.2f}")
        print(f"  y: {y.min():.2f} .. {y.max():.2f}")
        print(f"  z: {z.min():.2f} .. {z.max():.2f}")
        print(f"  range: {r.min():.2f} .. {r.max():.2f} м")
        for t in (10, 30, 50, 100, 150):
            print(f"  точек > {t} м: {int((r > t).sum())}")
        self.got = True


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else "/lidar/points"
    rclpy.init()
    n = Scan(topic)
    end = time.time() + 45
    try:
        while rclpy.ok() and not n.got and time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.3)
    except Exception as e:
        print(f"spin stopped: {e}", flush=True)
    finally:
        if not n.got:
            print("TIMEOUT: no messages", flush=True)
        try:
            n.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()