#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
class Preprocessor final : public rclcpp::Node {
public:
  Preprocessor();
private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  double voxel_size_, min_range_, max_range_, z_min_, z_max_, y_min_, y_max_;
  int sor_mean_k_; double sor_stddev_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};
