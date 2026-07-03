#ifndef PBL_LOCALIZATION__PBL_POSE_STAMPED_TF_BROADCASTER_HPP_
#define PBL_LOCALIZATION__PBL_POSE_STAMPED_TF_BROADCASTER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace pbl_pose_stamped_tf_broadcaster {

class pbl_pose_stamped_tf_broadcaster : public rclcpp::Node {
public:
  explicit pbl_pose_stamped_tf_broadcaster(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void publish_current_pose();

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string map_frame_id_;
  std::string base_frame_id_;
  std::string output_topic_;
  double publish_rate_hz_;
  double transform_timeout_sec_;
};

}  // namespace pbl_pose_stamped_tf_broadcaster

#endif  // PBL_LOCALIZATION__PBL_POSE_STAMPED_TF_BROADCASTER_HPP_
