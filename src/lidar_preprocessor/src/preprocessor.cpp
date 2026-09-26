#include "lidar_preprocessor/preprocessor.hpp"

#include <cmath>
#include <functional>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

Preprocessor::Preprocessor()
    : Node("pointcloud_preprocessor")
{
    voxel_size_ = declare_parameter("voxel_size", 0.10);
    min_range_ = declare_parameter("min_range", 1.0);
    max_range_ = declare_parameter("max_range", 180.0);

    z_min_ = declare_parameter("z_min", -2.0);
    z_max_ = declare_parameter("z_max", 5.0);

    y_min_ = declare_parameter("y_min", -15.0);
    y_max_ = declare_parameter("y_max", 15.0);

    sor_mean_k_ = declare_parameter("sor_mean_k", 16);
    sor_stddev_ = declare_parameter("sor_stddev", 1.5);

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lidar/points",
        rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile(),
        std::bind(&Preprocessor::callback, this, std::placeholders::_1)
    );

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/preprocessing/points",
        rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile()
    );
    RCLCPP_INFO(
        get_logger(),
        "LiDAR preprocessing node started"
    );
}

void Preprocessor::callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{       
    
    // ROS2 PointCloud2 -> PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr input(
        new pcl::PointCloud<pcl::PointXYZ>
    );

    pcl::fromROSMsg(*msg, *input);

    // -----------------------------------------
    // 1. Range + ROI filtering
    // -----------------------------------------

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(
        new pcl::PointCloud<pcl::PointXYZ>
    );

    filtered->reserve(input->size());

    const double min_distance_squared =
        min_range_ * min_range_;

    const double max_distance_squared =
        max_range_ * max_range_;

    for (const auto & point : *input)
    {
        // Удаляем NaN / Inf
        if (!std::isfinite(point.x) ||
            !std::isfinite(point.y) ||
            !std::isfinite(point.z))
        {
            continue;
        }

        const double distance_squared =
            point.x * point.x +
            point.y * point.y +
            point.z * point.z;

        // Ограничение дальности
        if (distance_squared < min_distance_squared ||
            distance_squared > max_distance_squared)
        {
            continue;
        }

        // Ограничение по высоте
        if (point.z < z_min_ ||
            point.z > z_max_)
        {
            continue;
        }

        // Ограничение по ширине
        if (point.y < y_min_ ||
            point.y > y_max_)
        {
            continue;
        }

        filtered->push_back(point);
    }

    // -----------------------------------------
    // 2. Voxel Grid
    // -----------------------------------------

    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud(
        new pcl::PointCloud<pcl::PointXYZ>
    );

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;

    voxel_filter.setInputCloud(filtered);

    voxel_filter.setLeafSize(
        voxel_size_,
        voxel_size_,
        voxel_size_
    );

    voxel_filter.filter(*voxel_cloud);

    // -----------------------------------------
    // 3. Statistical Outlier Removal
    // -----------------------------------------

    pcl::PointCloud<pcl::PointXYZ>::Ptr clean_cloud(
        new pcl::PointCloud<pcl::PointXYZ>
    );

    if (voxel_cloud->size() >
        static_cast<std::size_t>(sor_mean_k_ + 1))
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

        sor.setInputCloud(voxel_cloud);

        sor.setMeanK(sor_mean_k_);

        sor.setStddevMulThresh(sor_stddev_);

        sor.filter(*clean_cloud);
    }
    else
    {
        *clean_cloud = *voxel_cloud;
    }

    // -----------------------------------------
    // 4. PCL -> ROS2 PointCloud2
    // -----------------------------------------

    sensor_msgs::msg::PointCloud2 output;

    pcl::toROSMsg(
        *clean_cloud,
        output
    );

    output.header = msg->header;

    pub_->publish(output);
}

// =====================================================
// ROS2 ENTRY POINT
// =====================================================

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<Preprocessor>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}