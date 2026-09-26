#include <memory>
#include "rclcpp/rclcpp.hpp"

// ВНИМАНИЕ: Если внутри вашего оригинального файла 'pointcloud_preprocessor.cpp' 
// класс называется иначе или находится внутри namespace (например namespace lidar), 
// то измените шаблон ниже под вашу структуру кода.

class PointCloudPreprocessor : public rclcpp::Node
{
public:
    PointCloudPreprocessor() : Node("pointcloud_preprocessor")
    {
        // Базовый конструктор для запуска узла
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudPreprocessor>());
    rclcpp::shutdown();
    return 0;
}
