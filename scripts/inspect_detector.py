#!/usr/bin/env python3
import rclpy, time
from rclpy.node import Node

rclpy.init()
n = Node("inspect")
time.sleep(3)
for name, ns in n.get_node_names_and_namespaces():
    print(f"node: {ns}{name}")
# Subscriptions детектора через API
for node_name in ("/obstacle_detector",):
    try:
        info = n.get_publishers_info_by_topic("/obstacle_detection")
        print("publishers /obstacle_detection:")
        for i in info:
            print(f"  {i.node_name} reliable={i.qos_profile.reliability}")
    except Exception as e:
        print("err:", e)
n.destroy_node()
rclpy.shutdown()