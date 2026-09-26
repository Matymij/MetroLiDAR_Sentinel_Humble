#!/usr/bin/env python3
import csv, os, time
import rclpy
from rclpy.node import Node
from metro_lidar_msgs.msg import Benchmark
class BenchmarkNode(Node):
    def __init__(self):
        super().__init__('metro_benchmark'); self.declare_parameter('output_csv','/benchmark/benchmark.csv'); self.path=str(self.get_parameter('output_csv').value); os.makedirs(os.path.dirname(self.path) or '.',exist_ok=True); self.f=open(self.path,'w',newline='',encoding='utf-8'); self.w=csv.writer(self.f); self.w.writerow(['wall_time','processing_ms','fps','points','candidates','cpu_percent','rss_mb']); self.sub=self.create_subscription(Benchmark,'/benchmark',self.cb,20); self.count=0; self.t0=time.perf_counter(); self.create_timer(5.0,self.report)
    def cb(self,m): self.count+=1; self.w.writerow([time.time(),m.processing_time_ms,m.frame_rate_hz,m.point_count,m.candidate_count,m.cpu_percent,m.rss_mb]); self.f.flush()
    def report(self): self.get_logger().info(f'benchmark frames={self.count} rate={self.count/max(time.perf_counter()-self.t0,1e-6):.2f} Hz csv={self.path}')
def main(): rclpy.init(); n=BenchmarkNode(); rclpy.spin(n); n.f.close(); n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
