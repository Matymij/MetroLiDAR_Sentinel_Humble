#!/usr/bin/env python3
import glob, os, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import open3d as o3d
class Player(Node):
    def __init__(self):
        super().__init__('lidar_dataset_player'); self.declare_parameter('data_dir','/data'); self.declare_parameter('rate_hz',10.0); self.declare_parameter('loop',False)
        d=str(self.get_parameter('data_dir').value); self.loop=bool(self.get_parameter('loop').value); self.files=sorted(glob.glob(os.path.join(d,'*.pcd'))+glob.glob(os.path.join(d,'*.ply'))); self.i=0; self.pub=self.create_publisher(PointCloud2,'/lidar/points',10); self.timer=self.create_timer(1.0/max(float(self.get_parameter('rate_hz').value),0.1),self.tick); self.get_logger().info(f'Loaded {len(self.files)} PCD/PLY frames')
    def tick(self):
        if self.i>=len(self.files):
            if self.loop: self.i=0
            else: return
        path=self.files[self.i]; self.i+=1; pc=o3d.io.read_point_cloud(path); pts=np.asarray(pc.points,dtype=np.float32)
        if len(pts)==0: return
        msg=point_cloud2.create_cloud_xyz32(Header(stamp=self.get_clock().now().to_msg(),frame_id='lidar'),pts.tolist()); self.pub.publish(msg)
def main(): rclpy.init(); n=Player(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
