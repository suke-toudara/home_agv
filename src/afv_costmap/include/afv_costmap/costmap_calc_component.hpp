#ifndef COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_HPP_
#define COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_HPP_


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

namespace afv_costmap_calculator
{
class AfvCostmapCalculator : public rclcpp::Node
{
public:
  explicit AfvCostmapCalculator(const rclcpp::NodeOptions & options);

private:
  //publisher
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
  //subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  //function
  void initGridMap();
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
  
  boost::circular_buffer<sensor_msgs::msg::PointCloud2::SharedPtr> cloud_buffer_;
  boost::circular_buffer<sensor_msgs::msg::LaserScan::SharedPtr> scan_buffer_;
  grid_map::GridMap grid_map_;
  std::vector<geometry_msgs::msg::Point> transformScanPoints(
    const sensor_msgs::msg::LaserScan & scan,
    const geometry_msgs::msg::Pose & pose = geometry_msgs::msg::Pose()) const;
  void addPointsToGridMap(const std::vector<geometry_msgs::msg::Point> & points, const std::string & scan_layer_name);
  
  void publish();
  std::string output_topic_;
  double update_rate_;
  double resolution_;
  double laser_resolution_;
  rclcpp::Time timestamp_;
  int num_grids_;
  int laser_num_grids_;
  double range_max_;
  double forgetting_rate_;
  bool use_scan_;
  size_t scan_buffer_size_;
  std::string visualize_frame_id_;
  std::shared_ptr<data_buffer::PoseStampedDataBuffer> pose_buffer_;
  const geometry_msgs::msg::Pose getRelativePose(
    const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to) const;
};
}  // namespace robotx_costmap_calculator

#endif  //ROBOTX_COSTMAP_CALCULATOR_COSTMAP_COMPONENT_HPP_