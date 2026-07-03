#include "pbl_localization/pbl_pose_stamped_tf_broadcaster.hpp"

#include <chrono>
#include <functional>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pbl_pose_stamped_tf_broadcaster {

pbl_pose_stamped_tf_broadcaster::pbl_pose_stamped_tf_broadcaster(
  const rclcpp::NodeOptions & options)
: Node("pbl_pose_stamped_tf_broadcaster", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_, true),
  map_frame_id_(this->declare_parameter<std::string>("map_frame_id", "map")),
  base_frame_id_(this->declare_parameter<std::string>("base_frame_id", "base_link")),
  output_topic_(this->declare_parameter<std::string>("output_topic", "/localization/current_pose")),
  publish_rate_hz_(this->declare_parameter<double>("publish_rate_hz", 10.0)),
  transform_timeout_sec_(this->declare_parameter<double>("transform_timeout_sec", 0.1))
{
  tf_buffer_.setUsingDedicatedThread(true);

  if (publish_rate_hz_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "publish_rate_hz must be positive. Falling back to 10.0 Hz.");
    publish_rate_hz_ = 10.0;
  }
  if (transform_timeout_sec_ < 0.0) {
    RCLCPP_WARN(
      this->get_logger(), "transform_timeout_sec must be non-negative. Falling back to 0.1 s.");
    transform_timeout_sec_ = 0.1;
  }

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    output_topic_, rclcpp::QoS(10).reliable());

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_rate_hz_));
  publish_timer_ = this->create_wall_timer(
    period, std::bind(&pbl_pose_stamped_tf_broadcaster::publish_current_pose, this));

  RCLCPP_INFO(this->get_logger(), "pbl_pose_stamped_tf_broadcaster initialized");
  RCLCPP_INFO(this->get_logger(), "map_frame_id: %s", map_frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "base_frame_id: %s", base_frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "output_topic: %s", output_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "publish_rate_hz: %.3f", publish_rate_hz_);
  RCLCPP_INFO(
    this->get_logger(), "transform_timeout_sec: %.3f", transform_timeout_sec_);
}

void pbl_pose_stamped_tf_broadcaster::publish_current_pose()
{
  geometry_msgs::msg::PoseStamped pose_msg;
  try {
    const auto transform = tf_buffer_.lookupTransform(
      map_frame_id_, base_frame_id_, tf2::TimePointZero,
      tf2::durationFromSec(transform_timeout_sec_));

    pose_msg.header.stamp = transform.header.stamp;
    pose_msg.header.frame_id = map_frame_id_;
    pose_msg.pose.position.x = transform.transform.translation.x;
    pose_msg.pose.position.y = transform.transform.translation.y;
    pose_msg.pose.position.z = transform.transform.translation.z;
    pose_msg.pose.orientation = transform.transform.rotation;
    pose_pub_->publish(pose_msg);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Failed to lookup %s -> %s transform: %s", map_frame_id_.c_str(), base_frame_id_.c_str(),
      ex.what());
  }
}

}  // namespace pbl_pose_stamped_tf_broadcaster

RCLCPP_COMPONENTS_REGISTER_NODE(pbl_pose_stamped_tf_broadcaster::pbl_pose_stamped_tf_broadcaster)
