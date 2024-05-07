#include <rclcpp/rclcpp.hpp>
#include <afv_costmap/costmap_interpolation_component.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component =
    std::make_shared<afv_costmap::CostmapInterpolationComponent>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}