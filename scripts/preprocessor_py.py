#!/usr/bin/env python3
"""Python-версия preprocessor (замена C++ node).
Работает через numpy — быстрее, чем кажется, и главное — большие сообщения
от Python bag player проходят без потерь.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class Preprocessor(Node):
    def __init__(self):
        super().__init__("pointcloud_preprocessor")
        self.voxel_size = float(self.declare_parameter("voxel_size", 0.10).value)
        self.min_range = float(self.declare_parameter("min_range", 1.0).value)
        self.max_range = float(self.declare_parameter("max_range", 220.0).value)
        self.z_min = float(self.declare_parameter("z_min", -8.0).value)
        self.z_max = float(self.declare_parameter("z_max", 7.0).value)
        self.y_min = float(self.declare_parameter("y_min", -240.0).value)
        self.y_max = float(self.declare_parameter("y_max", 5.0).value)
        self.sor_k = int(self.declare_parameter("sor_mean_k", 10).value)
        self.sor_std = float(self.declare_parameter("sor_stddev", 2.5).value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )

        self.sub = self.create_subscription(
            PointCloud2, "/lidar/points", self.cb, qos)
        self.pub = self.create_publisher(
            PointCloud2, "/preprocessing/points", qos)

        self.frame = 0
        self.get_logger().info(
            f"Python preprocessor started: voxel={self.voxel_size} "
            f"range=[{self.min_range},{self.max_range}] "
            f"y=[{self.y_min},{self.y_max}] z=[{self.z_min},{self.z_max}]")

    def cb(self, msg):
        self.frame += 1
        try:
            arr = point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True)
            if arr.size == 0:
                return
            pts = np.stack(
                [arr['x'], arr['y'], arr['z']], axis=1).astype(np.float32)

            # Range + ROI filtering
            d2 = (pts * pts).sum(axis=1)
            mask = (d2 >= self.min_range ** 2) & (d2 <= self.max_range ** 2)
            mask &= (pts[:, 2] >= self.z_min) & (pts[:, 2] <= self.z_max)
            mask &= (pts[:, 1] >= self.y_min) & (pts[:, 1] <= self.y_max)
            pts = pts[mask]

            if pts.shape[0] == 0:
                return

            # Voxel grid (numpy hashing — быстрее open3d/pcl на 1 млн точек)
            vs = self.voxel_size
            keys = np.floor(pts / vs).astype(np.int64)
            # Уникальные ключи
            _, idx = np.unique(keys, axis=0, return_index=True)
            pts = pts[idx]

            # Statistical Outlier Removal (по k-ближайшим)
            if pts.shape[0] > self.sor_k + 1:
                from scipy.spatial import cKDTree
                tree = cKDTree(pts)
                d, _ = tree.query(pts, k=self.sor_k + 1, workers=-1)
                # d[:,0] — сама точка, d[:,1:] — соседи
                mean_d = d[:, 1:].mean(axis=1)
                global_mean = mean_d.mean()
                global_std = mean_d.std()
                thresh = global_mean + self.sor_std * global_std
                pts = pts[mean_d <= thresh]

            if pts.shape[0] == 0:
                return

            # Сборка PointCloud2 (x,y,z float32, point_step=12)
            out = PointCloud2()
            out.header = msg.header
            out.height = 1
            out.width = pts.shape[0]
            out.fields = []
            out.is_bigendian = False
            out.point_step = 12
            out.row_step = 12 * pts.shape[0]
            out.is_dense = True
            out.data = pts.astype(np.float32).tobytes()

            # Заполняем fields (для совместимости со старым детектором)
            from sensor_msgs.msg import PointField
            for i, name in enumerate(("x", "y", "z")):
                f = PointField()
                f.name = name
                f.offset = i * 4
                f.datatype = PointField.FLOAT32
                f.count = 1
                out.fields.append(f)

            self.pub.publish(out)

            if self.frame % 10 == 0:
                self.get_logger().info(
                    f"cb frame #{self.frame}, in={arr.size} → out={pts.shape[0]}")
        except Exception as e:
            self.get_logger().error(f"cb error: {e}")


def main():
    rclpy.init()
    n = Preprocessor()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()