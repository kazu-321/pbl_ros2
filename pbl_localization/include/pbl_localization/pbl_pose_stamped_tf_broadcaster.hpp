#ifndef PBL_LOCALIZATION__PBL_POSE_STAMPED_TF_BROADCASTER_HPP_
#define PBL_LOCALIZATION__PBL_POSE_STAMPED_TF_BROADCASTER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace pbl_pose_stamped_tf_broadcaster {

class pbl_pose_stamped_tf_broadcaster : public rclcpp::Node {
public:
  explicit pbl_pose_stamped_tf_broadcaster(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string map_frame_id_;
  std::string base_frame_id_;
  std::string pose_topic_;
  double transform_tolerance_sec_;
  bool publish_tf_;
};

}  // namespace pbl_pose_stamped_tf_broadcaster

#endif  // PBL_LOCALIZATION__PBL_POSE_STAMPED_TF_BROADCASTER_HPP_
