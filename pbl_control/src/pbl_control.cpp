// Copyright 2026 Kazusa Hashimoto
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pbl_control/pbl_control.hpp"

#include <algorithm>
#include <cmath>

namespace pbl_control {

pbl_control::pbl_control (const rclcpp::NodeOptions &node_options)
    : Node ("pbl_control", node_options),
      chassis_max_wheel_speed_rad_s_ (this->declare_parameter<double> ("chassis_max_wheel_speed_rad_s", 30.0)),
      chassis_acceleration_rate_s_ (this->declare_parameter<double> ("chassis_acceleration_rate_s", 1.0)),
            wheel_radius_ (this->declare_parameter<double> ("wheel_radius", 0.05)),
            wheel_separation_ (this->declare_parameter<double> ("wheel_separation", 0.46)),
            wheel_axle_x_ (this->declare_parameter<double> ("wheel_axle_x", -0.14)),
            odom_frame_id_ (this->declare_parameter<std::string> ("odom_frame_id", "odom")),
            base_frame_id_ (this->declare_parameter<std::string> ("base_frame_id", "base_link")),
            max_linear_speed_m_s_ (0.0),
      max_yaw_speed_rad_s_ (0.0),
      current_linear_speed_m_s_ (0.0),
      current_yaw_speed_rad_s_ (0.0),
      max_linear_acceleration_m_s2_ (0.0),
      max_yaw_acceleration_rad_s2_ (0.0),
      power_state_ (false),
    odom_initialized_ (false),
    last_joy_time_ (this->now ()),
    last_joint_state_time_ (this->now ()),
    odom_x_ (0.0),
    odom_y_ (0.0),
    odom_yaw_ (0.0),
    last_left_wheel_position_rad_ (0.0),
    last_right_wheel_position_rad_ (0.0) {
    if (chassis_acceleration_rate_s_ <= 0.0) {
        RCLCPP_WARN (this->get_logger (), "chassis_acceleration_rate_s must be positive. Falling back to 1.0 s.");
        chassis_acceleration_rate_s_ = 1.0;
    }

    if (wheel_radius_ <= 0.0) {
        RCLCPP_WARN (this->get_logger (), "wheel_radius must be positive. Falling back to 0.05 m.");
        wheel_radius_ = 0.05;
    }
    if (wheel_separation_ <= 0.0) {
        RCLCPP_WARN (this->get_logger (), "wheel_separation must be positive. Falling back to 0.46 m.");
        wheel_separation_ = 0.46;
    }

    max_linear_speed_m_s_         = wheel_radius_ * chassis_max_wheel_speed_rad_s_;
    max_yaw_speed_rad_s_          = (2.0 * max_linear_speed_m_s_) / wheel_separation_;
    max_linear_acceleration_m_s2_ = max_linear_speed_m_s_ / chassis_acceleration_rate_s_;
    max_yaw_acceleration_rad_s2_  = max_yaw_speed_rad_s_ / chassis_acceleration_rate_s_;
    joint_command_publisher_      = this->create_publisher<sensor_msgs::msg::JointState> ("/joint_commands", rclcpp::QoS (10));
    command_velocity_publisher_   = this->create_publisher<geometry_msgs::msg::TwistStamped> ("/command_velocity", rclcpp::QoS (10));
    odometry_publisher_           = this->create_publisher<nav_msgs::msg::Odometry> ("/odometry", rclcpp::QoS (10));
    power_publisher_              = this->create_publisher<std_msgs::msg::Bool> ("/power", rclcpp::QoS (10));
    joy_subscriber_               = this->create_subscription<sensor_msgs::msg::Joy> ("/controller/joy", rclcpp::QoS (10), std::bind (&pbl_control::joy_callback, this, std::placeholders::_1));
    joint_state_subscriber_       = this->create_subscription<sensor_msgs::msg::JointState> ("/joint_states", rclcpp::SensorDataQoS (), std::bind (&pbl_control::joint_state_callback, this, std::placeholders::_1));
    tf_broadcaster_               = std::make_shared<tf2_ros::TransformBroadcaster> (this);

    publish_power_state ();

    RCLCPP_INFO (this->get_logger (), "pbl_control node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "chassis_max_wheel_speed_rad_s: %.3f", chassis_max_wheel_speed_rad_s_);
    RCLCPP_INFO (this->get_logger (), "chassis_acceleration_rate_s: %.3f", chassis_acceleration_rate_s_);
    RCLCPP_INFO (this->get_logger (), "wheel_radius: %.3f", wheel_radius_);
    RCLCPP_INFO (this->get_logger (), "wheel_separation: %.3f", wheel_separation_);
    RCLCPP_INFO (this->get_logger (), "wheel_axle_x: %.3f", wheel_axle_x_);
    RCLCPP_INFO (this->get_logger (), "max_linear_speed_m_s: %.3f", max_linear_speed_m_s_);
    RCLCPP_INFO (this->get_logger (), "max_yaw_speed_rad_s: %.3f", max_yaw_speed_rad_s_);
    RCLCPP_INFO (this->get_logger (), "odom_frame_id: %s", odom_frame_id_.c_str ());
    RCLCPP_INFO (this->get_logger (), "base_frame_id: %s", base_frame_id_.c_str ());
    RCLCPP_INFO (this->get_logger (), "max_linear_acceleration_m_s2: %.3f", max_linear_acceleration_m_s2_);
    RCLCPP_INFO (this->get_logger (), "max_yaw_acceleration_rad_s2: %.3f", max_yaw_acceleration_rad_s2_);
}

double pbl_control::apply_deadzone (double value, double deadzone) {
    return (std::abs (value) < deadzone) ? 0.0 : value;
}

double pbl_control::normalize_pair (double value_a, double value_b, double value) {
    const double magnitude = std::abs (value_a) + std::abs (value_b);
    if (magnitude > 1.0) {
        return value / magnitude;
    }
    return value;
}

void pbl_control::publish_power_state () {
    std_msgs::msg::Bool power_msg;
    power_msg.data = power_state_;
    power_publisher_->publish (power_msg);
}

void pbl_control::publish_joint_commands (double linear_speed_m_s, double yaw_speed_rad_s) {
    const double right_wheel_speed_rad_s = std::clamp ((linear_speed_m_s + yaw_speed_rad_s * wheel_separation_ * 0.5) / wheel_radius_, -chassis_max_wheel_speed_rad_s_, chassis_max_wheel_speed_rad_s_);
    const double left_wheel_speed_rad_s  = std::clamp ((linear_speed_m_s - yaw_speed_rad_s * wheel_separation_ * 0.5) / wheel_radius_, -chassis_max_wheel_speed_rad_s_, chassis_max_wheel_speed_rad_s_);

    sensor_msgs::msg::JointState joint_commands_msg;
    joint_commands_msg.header.frame_id = "command/base_link";
    joint_commands_msg.header.stamp = this->now ();
    joint_commands_msg.name         = {"joint0", "joint1"};
    joint_commands_msg.position     = {0.0, 0.0};
    joint_commands_msg.velocity     = {right_wheel_speed_rad_s, left_wheel_speed_rad_s};
    joint_commands_msg.effort       = {0.0, 0.0};
    joint_command_publisher_->publish (joint_commands_msg);
}

void pbl_control::publish_command_velocity (double linear_speed_m_s, double yaw_speed_rad_s) {
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp    = this->now ();
    twist_msg.header.frame_id = "base_link";
    twist_msg.twist.linear.x  = linear_speed_m_s;
    twist_msg.twist.linear.y  = -yaw_speed_rad_s * wheel_axle_x_;
    twist_msg.twist.angular.z = yaw_speed_rad_s;
    command_velocity_publisher_->publish (twist_msg);
}

void pbl_control::publish_odometry (const rclcpp::Time &stamp, double linear_speed_m_s, double yaw_speed_rad_s) {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp    = stamp;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id   = base_frame_id_;
    odom_msg.pose.pose.position.x = odom_x_;
    odom_msg.pose.pose.position.y = odom_y_;
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion quaternion;
    quaternion.setRPY (0.0, 0.0, odom_yaw_);
    odom_msg.pose.pose.orientation.x = quaternion.x ();
    odom_msg.pose.pose.orientation.y = quaternion.y ();
    odom_msg.pose.pose.orientation.z = quaternion.z ();
    odom_msg.pose.pose.orientation.w = quaternion.w ();

    odom_msg.twist.twist.linear.x  = linear_speed_m_s;
    odom_msg.twist.twist.linear.y  = -yaw_speed_rad_s * wheel_axle_x_;
    odom_msg.twist.twist.angular.z = yaw_speed_rad_s;
    odometry_publisher_->publish (odom_msg);

    if (tf_broadcaster_ != nullptr) {
        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.header.stamp    = stamp;
        transform_msg.header.frame_id  = odom_frame_id_;
        transform_msg.child_frame_id   = base_frame_id_;
        transform_msg.transform.translation.x = odom_x_;
        transform_msg.transform.translation.y = odom_y_;
        transform_msg.transform.translation.z = 0.0;
        transform_msg.transform.rotation      = odom_msg.pose.pose.orientation;
        tf_broadcaster_->sendTransform (transform_msg);
    }
}

void pbl_control::joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::size_t right_wheel_index = msg->name.size ();
    std::size_t left_wheel_index  = msg->name.size ();

    for (std::size_t index = 0; index < msg->name.size (); ++index) {
        if (msg->name[index] == kRightWheelJointName) {
            right_wheel_index = index;
        }
        if (msg->name[index] == kLeftWheelJointName) {
            left_wheel_index = index;
        }
    }

    if (right_wheel_index == msg->name.size () || left_wheel_index == msg->name.size ()) {
        RCLCPP_WARN_THROTTLE (this->get_logger (), *this->get_clock (), 3000, "Could not find joint0/joint1 in /joint_states.");
        return;
    }

    const bool has_positions = right_wheel_index < msg->position.size () && left_wheel_index < msg->position.size ();
    const bool has_velocities = right_wheel_index < msg->velocity.size () && left_wheel_index < msg->velocity.size ();

    const bool has_stamp = msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0;
    const rclcpp::Time stamp = has_stamp ? rclcpp::Time (msg->header.stamp) : this->now ();

    if (!odom_initialized_) {
        last_joint_state_time_        = stamp;
        odom_initialized_            = true;
        last_right_wheel_position_rad_ = has_positions ? msg->position[right_wheel_index] : 0.0;
        last_left_wheel_position_rad_  = has_positions ? msg->position[left_wheel_index] : 0.0;
        publish_odometry (stamp, 0.0, 0.0);
        return;
    }

    const double dt = std::max ((stamp - last_joint_state_time_).seconds (), 0.0);
    last_joint_state_time_ = stamp;

    double right_wheel_delta_rad = 0.0;
    double left_wheel_delta_rad  = 0.0;

    if (has_positions) {
        right_wheel_delta_rad = msg->position[right_wheel_index] - last_right_wheel_position_rad_;
        left_wheel_delta_rad  = msg->position[left_wheel_index] - last_left_wheel_position_rad_;
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
    const double lateral_speed_m_s     = -yaw_speed_rad_s * wheel_axle_x_;
    const double delta_yaw_rad         = yaw_speed_rad_s * dt;
    const double yaw_midpoint          = odom_yaw_ + delta_yaw_rad * 0.5;

    odom_x_ += (std::cos (yaw_midpoint) * linear_speed_m_s - std::sin (yaw_midpoint) * lateral_speed_m_s) * dt;
    odom_y_ += (std::sin (yaw_midpoint) * linear_speed_m_s + std::cos (yaw_midpoint) * lateral_speed_m_s) * dt;
    odom_yaw_ = std::atan2 (std::sin (odom_yaw_ + delta_yaw_rad), std::cos (odom_yaw_ + delta_yaw_rad));

    publish_odometry (stamp, linear_speed_m_s, yaw_speed_rad_s);
}

void pbl_control::joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg) {
    const rclcpp::Time now = this->now ();
    const double       dt  = std::max ((now - last_joy_time_).seconds (), 0.0);
    last_joy_time_         = now;

    const auto axis_or_zero = [msg] (std::size_t index) {
        if (index >= msg->axes.size ()) {
            return 0.0;
        }
        return static_cast<double> (msg->axes[index]);
    };

    const auto button_or_zero = [msg] (std::size_t index) {
        if (index >= msg->buttons.size ()) {
            return 0;
        }
        return msg->buttons[index];
    };

    const double raw_turn_axis    = apply_deadzone (axis_or_zero (kLeftStickHorizontalAxis), 0.1);
    const double raw_forward_axis = apply_deadzone (axis_or_zero (kLeftStickVerticalAxis), 0.1);

    const double target_linear_speed = raw_forward_axis * max_linear_speed_m_s_;
    const double target_yaw_speed    = raw_turn_axis * max_yaw_speed_rad_s_;
    const double max_linear_delta    = max_linear_acceleration_m_s2_ * dt;
    const double max_yaw_delta       = max_yaw_acceleration_rad_s2_ * dt;

    if (target_linear_speed > current_linear_speed_m_s_) {
        current_linear_speed_m_s_ = std::min (current_linear_speed_m_s_ + max_linear_delta, target_linear_speed);
    } else {
        current_linear_speed_m_s_ = std::max (current_linear_speed_m_s_ - max_linear_delta, target_linear_speed);
    }

    if (target_yaw_speed > current_yaw_speed_rad_s_) {
        current_yaw_speed_rad_s_ = std::min (current_yaw_speed_rad_s_ + max_yaw_delta, target_yaw_speed);
    } else {
        current_yaw_speed_rad_s_ = std::max (current_yaw_speed_rad_s_ - max_yaw_delta, target_yaw_speed);
    }

    if (button_or_zero (kPowerOnButtonIndex) == 1) {
        power_state_ = true;
    }
    if (button_or_zero (kPowerOffButtonIndex) == 1) {
        power_state_ = false;
    }

    if (!power_state_) {
        current_linear_speed_m_s_ = 0.0;
        current_yaw_speed_rad_s_  = 0.0;
        publish_power_state ();
        publish_joint_commands (0.0, 0.0);
        publish_command_velocity (0.0, 0.0);
        return;
    }

    publish_power_state ();
    publish_joint_commands (current_linear_speed_m_s_, current_yaw_speed_rad_s_);
    publish_command_velocity (current_linear_speed_m_s_, current_yaw_speed_rad_s_);
}

}  // namespace pbl_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (pbl_control::pbl_control)