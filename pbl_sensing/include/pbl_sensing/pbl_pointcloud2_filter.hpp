#ifndef PBL_SENSING__PBL_POINTCLOUD2_FILTER_HPP_
#define PBL_SENSING__PBL_POINTCLOUD2_FILTER_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace pbl_sensing
{

class pbl_pointcloud2_filter : public rclcpp::Node
{
public:
  explicit pbl_pointcloud2_filter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct CollisionBox
  {
    std::string name;
    double center_x;
    double center_y;
    double center_z;
    double size_x;
    double size_y;
    double size_z;
  };

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publish_collision_markers();
  bool has_required_fields(const sensor_msgs::msg::PointCloud2 & msg) const;
  bool is_inside_box(double x, double y, double z, const CollisionBox & box) const;
  sensor_msgs::msg::PointCloud2 filter_self_points(const sensor_msgs::msg::PointCloud2 & cloud) const;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr collision_marker_pub_;
  rclcpp::TimerBase::SharedPtr collision_marker_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_id_;
  double transform_timeout_sec_;
  std::vector<CollisionBox> collision_boxes_;
};

}  // namespace pbl_sensing

#endif  // PBL_SENSING__PBL_POINTCLOUD2_FILTER_HPP_
