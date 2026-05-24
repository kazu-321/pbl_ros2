#ifndef PBL_SENSING__PBL_OCCUPANCY_GRID_PUBLISHER_HPP_
#define PBL_SENSING__PBL_OCCUPANCY_GRID_PUBLISHER_HPP_

#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace pbl_sensing
{

class pbl_occupancy_grid_publisher : public rclcpp::Node
{
public:
  explicit pbl_occupancy_grid_publisher(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void timer_callback();

  bool has_required_fields(const sensor_msgs::msg::PointCloud2 & msg) const;
  bool transform_pointcloud(
    const sensor_msgs::msg::PointCloud2 & in, sensor_msgs::msg::PointCloud2 & out,
    const std::string & target_frame, const rclcpp::Time & stamp) const;

  void initialize_grid_from_map(const sensor_msgs::msg::PointCloud2 & map_cloud);
  void build_static_occupancy_from_map(const sensor_msgs::msg::PointCloud2 & map_cloud);
  sensor_msgs::msg::PointCloud2 build_used_map_cloud(const sensor_msgs::msg::PointCloud2 & map_cloud) const;
  void append_cloud_buffer(const sensor_msgs::msg::PointCloud2 & cloud_in_map);
  sensor_msgs::msg::PointCloud2 take_cloud_buffer();
  void rebuild_occupancy_grid(const sensor_msgs::msg::PointCloud2 & cloud_in_map);

  std::size_t cell_index(std::size_t x_idx, std::size_t y_idx) const;
  bool world_to_cell(double x, double y, std::size_t & x_idx, std::size_t & y_idx) const;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr used_map_cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  mutable std::mutex mutex_;
  nav_msgs::msg::OccupancyGrid occupancy_grid_;
  std::vector<int8_t> static_occupancy_;
  std::vector<uint8_t> map_cell_present_;
  sensor_msgs::msg::PointCloud2 used_map_cloud_;
  sensor_msgs::msg::PointCloud2 cloud_buffer_;

  std::string map_topic_;
  std::string cloud_topic_;
  std::string map_frame_id_;
  double z_min_;
  double z_max_;
  double grid_resolution_;
  int occupied_count_full_score_;
  double transform_timeout_sec_;
  bool map_initialized_ = false;
  bool latest_cloud_ready_ = false;
};

}  // namespace pbl_sensing

#endif  // PBL_SENSING__PBL_OCCUPANCY_GRID_PUBLISHER_HPP_
