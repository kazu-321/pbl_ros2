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
      max_linear_speed_m_s_ (0.0),
      max_yaw_speed_rad_s_ (0.0),
      current_linear_speed_m_s_ (0.0),
      current_yaw_speed_rad_s_ (0.0),
      max_linear_acceleration_m_s2_ (0.0),
      max_yaw_acceleration_rad_s2_ (0.0),
      power_state_ (false),
      last_joy_time_ (this->now ()),
      last_joint_state_time_ (this->now ()) {
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
    power_publisher_              = this->create_publisher<std_msgs::msg::Bool> ("/power", rclcpp::QoS (10));
    joy_subscriber_               = this->create_subscription<sensor_msgs::msg::Joy> ("/controller/joy", rclcpp::QoS (10), std::bind (&pbl_control::joy_callback, this, std::placeholders::_1));

    publish_power_state ();

    RCLCPP_INFO (this->get_logger (), "pbl_control node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "chassis_max_wheel_speed_rad_s: %.3f", chassis_max_wheel_speed_rad_s_);
    RCLCPP_INFO (this->get_logger (), "chassis_acceleration_rate_s: %.3f", chassis_acceleration_rate_s_);
    RCLCPP_INFO (this->get_logger (), "wheel_radius: %.3f", wheel_radius_);
    RCLCPP_INFO (this->get_logger (), "wheel_separation: %.3f", wheel_separation_);
    RCLCPP_INFO (this->get_logger (), "wheel_axle_x: %.3f", wheel_axle_x_);
    RCLCPP_INFO (this->get_logger (), "max_linear_speed_m_s: %.3f", max_linear_speed_m_s_);
    RCLCPP_INFO (this->get_logger (), "max_yaw_speed_rad_s: %.3f", max_yaw_speed_rad_s_);
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
    joint_commands_msg.header.stamp    = this->now ();
    joint_commands_msg.name            = {"joint0", "joint1"};
    joint_commands_msg.position        = {0.0, 0.0};
    joint_commands_msg.velocity        = {right_wheel_speed_rad_s, left_wheel_speed_rad_s};
    joint_commands_msg.effort          = {0.0, 0.0};
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