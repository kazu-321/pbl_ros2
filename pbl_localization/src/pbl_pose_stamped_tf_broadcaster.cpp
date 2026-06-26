#include "pbl_localization/pbl_pose_stamped_tf_broadcaster.hpp"

#include <functional>

#include "rclcpp_components/register_node_macro.hpp"

namespace pbl_pose_stamped_tf_broadcaster {

pbl_pose_stamped_tf_broadcaster::pbl_pose_stamped_tf_broadcaster(
  const rclcpp::NodeOptions & options)
: Node("pbl_pose_stamped_tf_broadcaster", options),
  map_frame_id_(this->declare_parameter<std::string>("map_frame_id", "map")),
  base_frame_id_(this->declare_parameter<std::string>("base_frame_id", "base_link")),
  pose_topic_(this->declare_parameter<std::string>("pose_topic", "/localization/current_pose")),
  transform_tolerance_sec_(this->declare_parameter<double>("transform_tolerance_sec", 0.0)),
  publish_tf_(this->declare_parameter<bool>("publish_tf", true))
{
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic_, rclcpp::QoS(10).reliable(),
    std::bind(&pbl_pose_stamped_tf_broadcaster::pose_callback, this, std::placeholders::_1));

  if (publish_tf_) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

  RCLCPP_INFO(this->get_logger(), "pbl_pose_stamped_tf_broadcaster initialized");
  RCLCPP_INFO(this->get_logger(), "pose_topic: %s", pose_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "map_frame_id: %s", map_frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "base_frame_id: %s", base_frame_id_.c_str());
  RCLCPP_INFO(
    this->get_logger(), "transform_tolerance_sec: %.3f", transform_tolerance_sec_);
  RCLCPP_INFO(this->get_logger(), "publish_tf: %s", publish_tf_ ? "true" : "false");
}

void pbl_pose_stamped_tf_broadcaster::pose_callback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!publish_tf_ || !tf_broadcaster_) {
    return;
  }

  if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_id_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "pose frame_id is '%s' but expected '%s'. Publishing it as '%s' -> '%s'.",
      msg->header.frame_id.c_str(), map_frame_id_.c_str(), map_frame_id_.c_str(),
      base_frame_id_.c_str());
  }

  const bool has_stamp = msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0;
  const rclcpp::Time stamp = has_stamp ? rclcpp::Time(msg->header.stamp) : this->now();

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = stamp + rclcpp::Duration::from_seconds(transform_tolerance_sec_);
  tf_msg.header.frame_id = map_frame_id_;
  tf_msg.child_frame_id = base_frame_id_;
  tf_msg.transform.translation.x = msg->pose.position.x;
  tf_msg.transform.translation.y = msg->pose.position.y;
  tf_msg.transform.translation.z = msg->pose.position.z;
  tf_msg.transform.rotation = msg->pose.orientation;
  tf_broadcaster_->sendTransform(tf_msg);
}

}  // namespace pbl_pose_stamped_tf_broadcaster

RCLCPP_COMPONENTS_REGISTER_NODE(pbl_pose_stamped_tf_broadcaster::pbl_pose_stamped_tf_broadcaster)
