#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <afv_costmap/costmap_filter_component.hpp>
#include <string>
#include <unordered_map>
#include <vector>


namespace afv_costmap
{
AfvCostmapFilter::AfvCostmapFilter()
: Node("grid_map_filters"),filterChain_("grid_map::GridMap")
{
  if (!readParameters()) {
    return;
  }

  grid_map_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(inputTopic_, 1,std::bind(&AfvCostmapFilter::gridmapCallback, this, std::placeholders::_1));
  filter_map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>(outputTopic_, 1);

  // Setup filter chain.
  if (filterChain_.configure(
      filterChainParametersName_, this->get_node_logging_interface(),
      this->get_node_parameters_interface()))
  {
    RCLCPP_INFO(this->get_logger(), "Filter chain configured.");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Could not configure the filter chain!");
    rclcpp::shutdown();
    return;
  }
}

AfvCostmapFilter::~AfvCostmapFilter()
{
}

bool AfvCostmapFilter::readParameters()
{
  declare_parameter<std::string>("input_topic");
  declare_parameter("output_topic", std::string("output"));
  declare_parameter("filter_chain_parameter_name", std::string("filters"));
  if (get_parameter("input_topic", inputTopic_)) {
    RCLCPP_ERROR(this->get_logger(), "Could not read parameter `input_topic`.");
    return false;
  }
  get_parameter("output_topic", outputTopic_);
  get_parameter("filter_chain_parameter_name", filterChainParametersName_);
  return true;
}

void AfvCostmapFilter::gridmapCallback(const grid_map_msgs::msg::GridMap::SharedPtr message)
{
  grid_map::GridMap input_map;
  grid_map::GridMapRosConverter::fromMessage(*msg, input_map);
  grid_map::GridMap filtered_map;
  if (!filterChain_.update(input_map, filtered_map)) {
    RCLCPP_ERROR(this->get_logger(), "Could not update the grid map filter chain!");
    return;
  }

  auto filtered_map_msg = grid_map::GridMapRosConverter::toMessage(filtered_map);
  filter_map_pub_->publish(std::move(filtered_map_msg));
}
}