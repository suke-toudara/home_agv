#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "afv_costmap/afv_costmap_filter.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<afv_costmap::AfvCostmapFilter>());
  rclcpp::shutdown();
  return 0;
}