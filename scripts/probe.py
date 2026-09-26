#!/usr/bin/env python3
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
)
from rosidl_runtime_py.utilities import get_message


def make_qos(rel_str):
    rel = {
        "reliable": ReliabilityPolicy.RELIABLE,
        "best_effort": ReliabilityPolicy.BEST_EFFORT,
    }[rel_str]
    return QoSProfile(
        reliability=rel,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class Probe(Node):
    def __init__(self, topic, msg_type_str, rel_str):
        super().__init__("probe")
        self.got = 0
        self.msg_type = get_message(msg_type_str)
        qos = make_qos(rel_str)
        self.create_subscription(self.msg_type, topic, self.cb, qos)
        print(f"subscribed {topic} ({msg_type_str}) qos={rel_str}", flush=True)

    def cb(self, msg):
        if hasattr(msg, "width"):
            print(f"got PointCloud2: width={msg.width} height={msg.height} "
                  f"point_step={msg.point_step} frame={msg.header.frame_id}",
                  flush=True)
        else:
            print(f"got: {type(msg).__name__}", flush=True)
            print(msg, flush=True)
        self.got += 1


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else "/lidar/points"
    msg_type = sys.argv[2] if len(sys.argv) > 2 else "sensor_msgs/msg/PointCloud2"
    rel = sys.argv[3] if len(sys.argv) > 3 else "best_effort"

    print(f"starting probe on {topic}...", flush=True)
    rclpy.init()
    n = Probe(topic, msg_type, rel)
    end = time.time() + 45
    try:
        while rclpy.ok() and n.got < 1 and time.time() < end:
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