#include "pbl_localization/pbl_wheel_odometry.hpp"

#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include <algorithm>
#include <cmath>

namespace pbl_wheel_odometry {

pbl_wheel_odometry::pbl_wheel_odometry (const rclcpp::NodeOptions &options)
    : Node ("pbl_wheel_odometry", options),
      odom_frame_id_ (this->declare_parameter<std::string> ("odom_frame_id", "odom")),
      base_frame_id_ (this->declare_parameter<std::string> ("base_frame_id", "base_link")),
      wheel_radius_ (this->declare_parameter<double> ("wheel_radius", 0.05)),
      wheel_separation_ (this->declare_parameter<double> ("wheel_separation", 0.46)),
      publish_tf_ (this->declare_parameter<bool> ("publish_tf", false)) {
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry> ("/odometry", 10);
    if (publish_tf_) {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster> (this);
    }
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState> ("/joint_states", rclcpp::SensorDataQoS (), std::bind (&pbl_wheel_odometry::joint_state_callback, this, std::placeholders::_1));

    RCLCPP_INFO (this->get_logger (), "pbl_wheel_odometry initialized");
    RCLCPP_INFO (this->get_logger (), "wheel_radius: %.3f", wheel_radius_);
    RCLCPP_INFO (this->get_logger (), "wheel_separation: %.3f", wheel_separation_);
    RCLCPP_INFO (this->get_logger (), "odom_frame_id: %s", odom_frame_id_.c_str ());
    RCLCPP_INFO (this->get_logger (), "base_frame_id: %s", base_frame_id_.c_str ());
    RCLCPP_INFO (this->get_logger (), "publish_tf: %s", publish_tf_ ? "true" : "false");
}

void pbl_wheel_odometry::joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::size_t right_wheel_index = msg->name.size ();
    std::size_t left_wheel_index  = msg->name.size ();
    for (std::size_t i = 0; i < msg->name.size (); ++i) {
        if (msg->name[i] == "joint0") {
            right_wheel_index = i;
        }
        if (msg->name[i] == "joint1") {
            left_wheel_index = i;
        }
    }
    if (right_wheel_index == msg->name.size () || left_wheel_index == msg->name.size ()) {
        RCLCPP_WARN_THROTTLE (this->get_logger (), *this->get_clock (), 3000, "Could not find joint0/joint1 in /joint_states.");
        return;
    }

    const bool         has_positions  = right_wheel_index < msg->position.size () && left_wheel_index < msg->position.size ();
    const bool         has_velocities = right_wheel_index < msg->velocity.size () && left_wheel_index < msg->velocity.size ();
    const bool         has_stamp      = msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0;
    const rclcpp::Time stamp          = has_stamp ? rclcpp::Time (msg->header.stamp) : this->now ();

    if (!odom_initialized_) {
        last_joint_state_time_         = stamp;
        odom_initialized_              = true;
        last_right_wheel_position_rad_ = has_positions ? msg->position[right_wheel_index] : 0.0;
        last_left_wheel_position_rad_  = has_positions ? msg->position[left_wheel_index] : 0.0;
        publish_odometry (stamp, 0.0, 0.0);
        return;
    }

    const double dt        = std::max ((stamp - last_joint_state_time_).seconds (), 0.0);
    last_joint_state_time_ = stamp;

    double right_wheel_delta_rad = 0.0;
    double left_wheel_delta_rad  = 0.0;
    if (has_positions) {
        right_wheel_delta_rad          = msg->position[right_wheel_index] - last_right_wheel_position_rad_;
        left_wheel_delta_rad           = msg->position[left_wheel_index] - last_left_wheel_position_rad_;
        last_right_wheel_position_rad_ = msg->position[right_wheel_index];
        last_left_wheel_position_rad_  = msg->position[left_wheel_index];
    } else if (has_velocities && dt > 0.0) {
        right_wheel_delta_rad = msg->velocity[right_wheel_index] * dt;
        left_wheel_delta_rad  = msg->velocity[left_wheel_index] * dt;
    } else {
        publish_odometry (stamp, 0.0, 0.0);
        return;
    }

    const double right_wheel_speed_m_s = (dt > 0.0) ? (right_wheel_delta_rad * wheel_radius_ / dt) : 0.0;
    const double left_wheel_speed_m_s  = (dt > 0.0) ? (left_wheel_delta_rad * wheel_radius_ / dt) : 0.0;
    const double linear_speed_m_s      = 0.5 * (left_wheel_speed_m_s + right_wheel_speed_m_s);
    const double yaw_speed_rad_s       = (right_wheel_speed_m_s - left_wheel_speed_m_s) / wheel_separation_;
    const double delta_yaw_rad         = yaw_speed_rad_s * dt;
    const double yaw_midpoint          = odom_yaw_ + delta_yaw_rad * 0.5;
    odom_x_ += std::cos (yaw_midpoint) * linear_speed_m_s * dt;
    odom_y_ += std::sin (yaw_midpoint) * linear_speed_m_s * dt;
    odom_yaw_ = std::atan2 (std::sin (odom_yaw_ + delta_yaw_rad), std::cos (odom_yaw_ + delta_yaw_rad));
    publish_odometry (stamp, linear_speed_m_s, yaw_speed_rad_s);
}

void pbl_wheel_odometry::publish_odometry (const rclcpp::Time &stamp, double linear_speed_m_s, double yaw_speed_rad_s) {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp         = stamp;
    odom_msg.header.frame_id      = odom_frame_id_;
    odom_msg.child_frame_id       = base_frame_id_;
    odom_msg.pose.pose.position.x = odom_x_;
    odom_msg.pose.pose.position.y = odom_y_;
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY (0.0, 0.0, odom_yaw_);
    odom_msg.pose.pose.orientation.x = q.x ();
    odom_msg.pose.pose.orientation.y = q.y ();
    odom_msg.pose.pose.orientation.z = q.z ();
    odom_msg.pose.pose.orientation.w = q.w ();
    odom_msg.twist.twist.linear.x    = linear_speed_m_s;
    odom_msg.twist.twist.angular.z   = yaw_speed_rad_s;
    odom_pub_->publish (odom_msg);

    if (!publish_tf_ || !tf_broadcaster_) {
        return;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp            = stamp;
    tf_msg.header.frame_id         = odom_frame_id_;
    tf_msg.child_frame_id          = base_frame_id_;
    tf_msg.transform.translation.x = odom_x_;
    tf_msg.transform.translation.y = odom_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation      = odom_msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform (tf_msg);
}

}  // namespace pbl_wheel_odometry

RCLCPP_COMPONENTS_REGISTER_NODE (pbl_wheel_odometry::pbl_wheel_odometry)
