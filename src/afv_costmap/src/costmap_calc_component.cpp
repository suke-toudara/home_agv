#include <chrono>
#include <data_buffer/data_buffer_base.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <memory>
#include <rclcpp_components/register_node_macro.hpp>
#include <afv_costmap/costmap_calc_component.hpp>
#include <string>
#include <vector>
#ifdef USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace afv_costmap_calculator
{
AfvCostmapCalculator::AfvCostmapCalculator(const rclcpp::NodeOptions & options)
: Node("afv_costmap_calculator", options)
{
  std::string points_raw_topic;
  std::string laserscan_raw_topic;
  std::string current_pose_topic;

  declare_parameter<std::string>("laserscan_raw_topic", "/perception/pointcloud_to_laserscan_node/output");
  declare_parameter<std::string>("current_pose_topic", "/current_pose");
  declare_parameter("resolution", 1.0);
  declare_parameter("num_grids", 20);
  declare_parameter("range_max", 20.0);
  declare_parameter("visualize_frame_id", "map");
  declare_parameter("buffer_length", 5.0);
  declare_parameter("scan_buffer_size", 2);
  declare_parameter("forgetting_rate", 0.6);
  declare_parameter<bool>("use_scan", true);
  
  get_parameter("laserscan_raw_topic", laserscan_raw_topic);
  get_parameter("current_pose_topic", current_pose_topic);
  get_parameter("resolution", resolution_);
  get_parameter("num_grids", num_grids_);
  get_parameter("range_max", range_max_);
  get_parameter("visualize_frame_id", visualize_frame_id_);
  double buffer_length;
  get_parameter("buffer_length", buffer_length);
  get_parameter("scan_buffer_size", scan_buffer_size_);
  get_parameter("forgetting_rate", forgetting_rate_);
  get_parameter("use_scan", use_scan_);

  
  //publisher
  grid_map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>("grid_map", 1);
  //subscriber
  laserscan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    laserscan_raw_topic, 10,
    std::bind(&AfvCostmapCalculator::scanCallback, this, std::placeholders::_1));
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    current_pose_topic, 10,
    std::bind(&AfvCostmapCalculator::poseCallback, this, std::placeholders::_1));
  //buffer
  cloud_buffer_ = boost::circular_buffer<sensor_msgs::msg::PointCloud2::SharedPtr>(scan_buffer_size_);
  scan_buffer_  = boost::circular_buffer<sensor_msgs::msg::LaserScan::SharedPtr>(scan_buffer_size_);
  std::string key;
  pose_buffer_ = std::make_shared<data_buffer::PoseStampedDataBuffer>(get_clock(), key, buffer_length);
  
  initGridMap();
}


void AfvCostmapCalculator::initGridMap()
{
  grid_map_.setFrameId("base_link");
  //grid_map_.setFrameId("map");
  grid_map_.setGeometry(
    grid_map::Length(resolution_ * num_grids_, resolution_ * num_grids_), resolution_,
    grid_map::Position(0.0, 0.0));
}

void AfvCostmapCalculator::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  pose_buffer_->addData(*pose);
  return;
}


void AfvCostmapCalculator::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  scan_buffer_.push_back(scan);
  geometry_msgs::msg::PoseStamped scan_pose;
  if (!pose_buffer_->queryData(scan->header.stamp, scan_pose)) {
    return;
  }

  for (size_t j = 0; j < scan_buffer_.size(); j++) {
    std::stringstream ss;
    ss << j;
    std::string scan_layer_name("scan_layer" + ss.str());
    //最新のトピックは位置の変換いらない
    if (j == scan_buffer_.size() - 1) {
      addPointsToGridMap(transformScanPoints(*scan_buffer_[j]), scan_layer_name);
    //以前のトピックは位置の変換いる
    } else {
      geometry_msgs::msg::PoseStamped prev_scan_pose;
      if (!pose_buffer_->queryData(scan_buffer_[j]->header.stamp, prev_scan_pose)) {
        return;
      }
      addPointsToGridMap(
        transformScanPoints(*scan_buffer_[j], getRelativePose(scan_pose.pose, prev_scan_pose.pose)),scan_layer_name);
    }
  }
  publish();
  return;
}

//
std::vector<geometry_msgs::msg::Point> AfvCostmapCalculator::transformScanPoints(
  const sensor_msgs::msg::LaserScan & scan, const geometry_msgs::msg::Pose & pose) const
{
  std::vector<geometry_msgs::msg::Point> scan_point;
  Eigen::Matrix3d scan_rotation_matrix;
  scan_rotation_matrix = quaternion_operation::getRotationMatrix(pose.orientation);
  for (int i = 0; i < static_cast<int>(scan.ranges.size()); i++) {
    if (range_max_ >= scan.ranges[i]) {
      double theta = scan.angle_min + scan.angle_increment * static_cast<double>(i);
      Eigen::VectorXd v(3);
      v(0) = scan.ranges[i] * std::cos(theta);
      v(1) = scan.ranges[i] * std::sin(theta);
      v(2) = 0;
      //scanデータに位置と角度を補正
      v = scan_rotation_matrix * v;
      v(0) = v(0) + pose.position.x;
      v(1) = v(1) + pose.position.y;
      v(2) = v(2) + pose.position.z;
      geometry_msgs::msg::Point transformed;
      transformed.x = v(0);
      transformed.y = v(1);
      transformed.z = v(2);
      scan_point.emplace_back(transformed);
    }
  }
  return scan_point;
}

   
void AfvCostmapCalculator::addPointsToGridMap(const std::vector<geometry_msgs::msg::Point> & points, const std::string & scan_layer_name)
{
  grid_map_.add(scan_layer_name, 0.0);
  for (const auto & point : points) {
    //scanの中心から一定範囲costを追加する。
    for (grid_map::CircleIterator iterator(grid_map_, grid_map::Position(point.x, point.y), resolution_ * 0.5);
         !iterator.isPastEnd(); ++iterator) {
      if (std::isnan(grid_map_.at(scan_layer_name, *iterator))) {
        grid_map_.at(scan_layer_name, *iterator) = 0.0;
      } else {
        if (grid_map_.at(scan_layer_name, *iterator) < 1.0) {
          grid_map_.at(scan_layer_name, *iterator) = grid_map_.at(scan_layer_name, *iterator) + 0.1;
        }
      }
    }
  }
}

void AfvCostmapCalculator::publish()
{
  auto msg = grid_map::GridMapRosConverter::toMessage(grid_map_);
  msg->header.stamp = get_clock()->now();
  grid_map_pub_->publish(std::move(msg));
}

const geometry_msgs::msg::Pose AfvCostmapCalculator::getRelativePose(
  const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to) const
{
  geometry_msgs::msg::Transform from_translation;
  {
    from_translation.translation.x = from.position.x;
    from_translation.translation.y = from.position.y;
    from_translation.translation.z = from.position.z;
    from_translation.rotation = from.orientation;
  }

  tf2::Transform from_tf;
  {
    tf2::fromMsg(from_translation, from_tf);
  }

  geometry_msgs::msg::Transform to_translation;
  {
    to_translation.translation.x = to.position.x;
    to_translation.translation.y = to.position.y;
    to_translation.translation.z = to.position.z;
    to_translation.rotation = to.orientation;
  }

  tf2::Transform to_tf;
  {
    tf2::fromMsg(to_translation, to_tf);
  }

  tf2::Transform tf_delta = from_tf.inverse() * to_tf;

  geometry_msgs::msg::Pose ret;
  {
    tf2::toMsg(tf_delta, ret);
  }
  return ret;
}

}