#ifndef PBL_LOCALIZATION__PBL_GLOBAL_LOCALIZATION_HPP_
#define PBL_LOCALIZATION__PBL_GLOBAL_LOCALIZATION_HPP_

#include <deque>
#include <optional>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace pbl_localization {

class pbl_global_localization : public rclcpp::Node {
public:
  explicit pbl_global_localization(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using PointT = pcl::PointXYZ;

  struct BufferedCloud
  {
    pcl::PointCloud<PointT>::Ptr cloud;
    rclcpp::Time stamp;
  };

  void map_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void source_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void alignment_timer_callback();
  bool try_align(const char * trigger_reason);
  void publish_map_to_odom(const Eigen::Isometry3d & map_to_odom, const rclcpp::Time & stamp);

  static Eigen::Isometry3d pose_to_transform(const geometry_msgs::msg::Pose & pose);
  static std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> parse_initialpose_2d(
    const std::string & initialpose_2d, const std::string & frame_id);
  bool is_inside_map_xy(const Eigen::Vector3d & map_position) const;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr source_cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_sub_;
  rclcpp::TimerBase::SharedPtr alignment_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  pcl::PointCloud<PointT>::Ptr map_cloud_;
  std::deque<BufferedCloud> source_cloud_buffer_;
  geometry_msgs::msg::PoseWithCovarianceStamped latest_initial_pose_;

  std::mutex mutex_;

  std::string map_frame_id_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string map_cloud_topic_;
  std::string source_cloud_topic_;
  std::string initial_pose_topic_;
  double source_buffer_duration_sec_;
  double alignment_period_sec_;
  double ndt_resolution_;
  double ndt_step_size_;
  double ndt_transformation_epsilon_;
  int ndt_max_iterations_;
  double voxel_leaf_size_;
  double transform_tolerance_sec_;
  double map_xy_margin_m_;
  bool publish_tf_;
  bool wait_initialpose_;
  std::string initialpose_2d_;

  bool has_map_cloud_ = false;
  bool has_initial_pose_ = false;
  bool has_valid_alignment_ = false;
  bool force_initial_alignment_ = false;
  Eigen::Isometry3d last_map_to_odom_ = Eigen::Isometry3d::Identity();
  Eigen::Vector2d map_xy_min_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d map_xy_max_ = Eigen::Vector2d::Zero();
  bool has_map_xy_bounds_ = false;
};

}  // namespace pbl_localization

#endif  // PBL_LOCALIZATION__PBL_GLOBAL_LOCALIZATION_HPP_
