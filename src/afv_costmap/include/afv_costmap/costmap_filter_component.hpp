
// Headers in ROS
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>
#include <data_buffer/data_buffer_base.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs_data_buffer/pose_stamped_data_buffer.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <memory>
#include <pcl_apps_msgs/msg/polygon_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>



//追加
#include <grid_map_ros/grid_map_ros.hpp>
#include <filters/filter_chain.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>


namespace afv_costmap
{
class AfvCostmapFilter : public rclcpp::Node
{
public:
  AfvCostmapFilter();
  virtual ~AfvCostmapFilter();

  bool readParameters();
  void gridmapCallback(const grid_map_msgs::msg::GridMap::SharedPtr message);

private:
  std::string inputTopic_;
  std::string outputTopic_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr filter_map_pub_;
  filters::FilterChain<grid_map::GridMap> filterChain_;
  std::string filterChainParametersName_;
};
} 
#endif 