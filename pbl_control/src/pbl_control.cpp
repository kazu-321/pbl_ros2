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
#include <chrono>
#include <cmath>

namespace pbl_control {

pbl_control::pbl_control (const rclcpp::NodeOptions &node_options)
    : Node ("pbl_control", node_options),
      chassis_max_wheel_speed_rad_s_ (this->declare_parameter<double> ("chassis_max_wheel_speed_rad_s", 30.0)),
      chassis_acceleration_rate_s_ (this->declare_parameter<double> ("chassis_acceleration_rate_s", 1.0)),
      wheel_radius_ (this->declare_parameter<double> ("wheel_radius", 0.05)),
      wheel_separation_ (this->declare_parameter<double> ("wheel_separation", 0.46)),
      wheel_axle_x_ (this->declare_parameter<double> ("wheel_axle_x", -0.14)),
      configured_max_linear_speed_m_s_ (this->declare_parameter<double> ("max_linear_speed_m_s", 0.0)),
      configured_max_yaw_speed_rad_s_ (this->declare_parameter<double> ("max_yaw_speed_rad_s", 0.0)),
      max_linear_speed_m_s_ (0.0),
      max_yaw_speed_rad_s_ (0.0),
      current_linear_speed_m_s_ (0.0),
      current_yaw_speed_rad_s_ (0.0),
      max_linear_acceleration_m_s2_ (0.0),
      max_yaw_acceleration_rad_s2_ (0.0),
      power_state_ (false),
      auto_mode_state_ (false),
      last_joy_time_ (this->now ()),
      last_update_time_ (this->now ()),
      last_auto_command_time_ (this->now ()),
      target_linear_speed_m_s_ (0.0),
      target_yaw_speed_rad_s_ (0.0),
      auto_target_linear_speed_m_s_ (0.0),
      auto_target_yaw_speed_rad_s_ (0.0),
      control_rate_hz_ (this->declare_parameter<double> ("control_rate_hz", 50.0)),
      auto_command_timeout_sec_ (this->declare_parameter<double> ("auto_command_timeout_sec", 0.5)) {
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

    const double theoretical_max_linear_speed_m_s = wheel_radius_ * chassis_max_wheel_speed_rad_s_;
    const double theoretical_max_yaw_speed_rad_s  = (2.0 * theoretical_max_linear_speed_m_s) / wheel_separation_;

    max_linear_speed_m_s_ = theoretical_max_linear_speed_m_s;
    if (std::isfinite (configured_max_linear_speed_m_s_) && configured_max_linear_speed_m_s_ > 0.0) {
        max_linear_speed_m_s_ = std::min (configured_max_linear_speed_m_s_, theoretical_max_linear_speed_m_s);
    }

    max_yaw_speed_rad_s_ = theoretical_max_yaw_speed_rad_s;
    if (std::isfinite (configured_max_yaw_speed_rad_s_) && configured_max_yaw_speed_rad_s_ > 0.0) {
        max_yaw_speed_rad_s_ = std::min (configured_max_yaw_speed_rad_s_, theoretical_max_yaw_speed_rad_s);
    }

    max_linear_acceleration_m_s2_ = max_linear_speed_m_s_ / chassis_acceleration_rate_s_;
    max_yaw_acceleration_rad_s2_  = max_yaw_speed_rad_s_ / chassis_acceleration_rate_s_;
    joint_command_publisher_      = this->create_publisher<sensor_msgs::msg::JointState> ("/joint_commands", rclcpp::QoS (10));
    command_velocity_publisher_   = this->create_publisher<geometry_msgs::msg::TwistStamped> ("/command_velocity", rclcpp::QoS (10));
    power_publisher_              = this->create_publisher<std_msgs::msg::Bool> ("/power", rclcpp::QoS (10));
    auto_publisher_               = this->create_publisher<std_msgs::msg::Bool> ("/is_auto", rclcpp::QoS (10));
    joy_subscriber_               = this->create_subscription<sensor_msgs::msg::Joy> ("/controller/joy", rclcpp::QoS (10), std::bind (&pbl_control::joy_callback, this, std::placeholders::_1));
    cmd_vel_smoothed_sub_         = this->create_subscription<geometry_msgs::msg::Twist> ("/cmd_vel", rclcpp::QoS (10), std::bind (&pbl_control::cmd_vel_smoothed_callback, this, std::placeholders::_1));

    if (control_rate_hz_ <= 0.0) {
        RCLCPP_WARN (this->get_logger (), "control_rate_hz must be positive. Falling back to 50.0 Hz.");
        control_rate_hz_ = 50.0;
    }
    if (auto_command_timeout_sec_ <= 0.0) {
        RCLCPP_WARN (this->get_logger (), "auto_command_timeout_sec must be positive. Falling back to 0.5 s.");
        auto_command_timeout_sec_ = 0.5;
    }

    const auto control_period = std::chrono::duration_cast<std::chrono::nanoseconds> (std::chrono::duration<double> (1.0 / control_rate_hz_));
    control_timer_            = this->create_wall_timer (control_period, std::bind (&pbl_control::control_timer_callback, this));
    state_timer_              = this->create_wall_timer (std::chrono::seconds (1), std::bind (&pbl_control::publish_state_topics, this));

    publish_state_topics ();

    RCLCPP_INFO (this->get_logger (), "pbl_control node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "chassis_max_wheel_speed_rad_s: %.3f", chassis_max_wheel_speed_rad_s_);
    RCLCPP_INFO (this->get_logger (), "chassis_acceleration_rate_s: %.3f", chassis_acceleration_rate_s_);
    RCLCPP_INFO (this->get_logger (), "wheel_radius: %.3f", wheel_radius_);
    RCLCPP_INFO (this->get_logger (), "wheel_separation: %.3f", wheel_separation_);
    RCLCPP_INFO (this->get_logger (), "wheel_axle_x: %.3f", wheel_axle_x_);
    RCLCPP_INFO (this->get_logger (), "configured_max_linear_speed_m_s: %.3f (<= 0 means theoretical limit)", configured_max_linear_speed_m_s_);
    RCLCPP_INFO (this->get_logger (), "configured_max_yaw_speed_rad_s: %.3f (<= 0 means theoretical limit)", configured_max_yaw_speed_rad_s_);
    RCLCPP_INFO (this->get_logger (), "theoretical_max_linear_speed_m_s: %.3f", theoretical_max_linear_speed_m_s);
    RCLCPP_INFO (this->get_logger (), "theoretical_max_yaw_speed_rad_s: %.3f", theoretical_max_yaw_speed_rad_s);
    RCLCPP_INFO (this->get_logger (), "max_linear_speed_m_s: %.3f", max_linear_speed_m_s_);
    RCLCPP_INFO (this->get_logger (), "max_yaw_speed_rad_s: %.3f", max_yaw_speed_rad_s_);
    RCLCPP_INFO (this->get_logger (), "max_linear_acceleration_m_s2: %.3f", max_linear_acceleration_m_s2_);
    RCLCPP_INFO (this->get_logger (), "max_yaw_acceleration_rad_s2: %.3f", max_yaw_acceleration_rad_s2_);
    RCLCPP_INFO (this->get_logger (), "control_rate_hz: %.3f", control_rate_hz_);
    RCLCPP_INFO (this->get_logger (), "auto_command_timeout_sec: %.3f", auto_command_timeout_sec_);
    publish_auto_state ();
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

void pbl_control::publish_auto_state () {
    std_msgs::msg::Bool auto_msg;
    auto_msg.data = auto_mode_state_;
    auto_publisher_->publish (auto_msg);
}

void pbl_control::publish_state_topics () {
    publish_power_state ();
    publish_auto_state ();
}

void pbl_control::publish_joint_commands (double linear_speed_m_s, double yaw_speed_rad_s) {
    const double right_wheel_speed_rad_s = std::clamp ((linear_speed_m_s + yaw_speed_rad_s * wheel_separation_ * 0.5) / wheel_radius_, -chassis_max_wheel_speed_rad_s_, chassis_max_wheel_speed_rad_s_);
    const double left_wheel_speed_rad_s  = std::clamp ((linear_speed_m_s - yaw_speed_rad_s * wheel_separation_ * 0.5) / wheel_radius_, -chassis_max_wheel_speed_rad_s_, chassis_max_wheel_speed_rad_s_);

    sensor_msgs::msg::JointState joint_commands_msg;
    joint_commands_msg.header.frame_id = "command/base_link";
    joint_commands_msg.header.stamp    = this->now ();
    joint_commands_msg.name            = {"wheel_left", "wheel_right"};
    joint_commands_msg.position        = {0.0, 0.0};
    joint_commands_msg.velocity        = {left_wheel_speed_rad_s, right_wheel_speed_rad_s};
    joint_commands_msg.effort          = {0.0, 0.0};
    joint_command_publisher_->publish (joint_commands_msg);
}

void pbl_control::publish_command_velocity (double linear_speed_m_s, double yaw_speed_rad_s) {
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp    = this->now ();
    twist_msg.header.frame_id = "base_link";
    twist_msg.twist.linear.x  = linear_speed_m_s;
    twist_msg.twist.linear.y  = 0.0;
    twist_msg.twist.angular.z = yaw_speed_rad_s;
    command_velocity_publisher_->publish (twist_msg);
}

void pbl_control::cmd_vel_smoothed_callback (const geometry_msgs::msg::Twist::SharedPtr msg) {
    auto_target_linear_speed_m_s_ = std::clamp (msg->linear.x, -max_linear_speed_m_s_, max_linear_speed_m_s_);
    auto_target_yaw_speed_rad_s_  = std::clamp (msg->angular.z, -max_yaw_speed_rad_s_, max_yaw_speed_rad_s_);
    last_auto_command_time_       = this->now ();
}

void pbl_control::update_motion (double dt) {
    const double max_linear_delta = max_linear_acceleration_m_s2_ * dt;
    const double max_yaw_delta    = max_yaw_acceleration_rad_s2_ * dt;

    if (target_linear_speed_m_s_ > current_linear_speed_m_s_) {
        current_linear_speed_m_s_ = std::min (current_linear_speed_m_s_ + max_linear_delta, target_linear_speed_m_s_);
    } else {
        current_linear_speed_m_s_ = std::max (current_linear_speed_m_s_ - max_linear_delta, target_linear_speed_m_s_);
    }

    if (target_yaw_speed_rad_s_ > current_yaw_speed_rad_s_) {
        current_yaw_speed_rad_s_ = std::min (current_yaw_speed_rad_s_ + max_yaw_delta, target_yaw_speed_rad_s_);
    } else {
        current_yaw_speed_rad_s_ = std::max (current_yaw_speed_rad_s_ - max_yaw_delta, target_yaw_speed_rad_s_);
    }
}

void pbl_control::control_timer_callback () {
    const rclcpp::Time now = this->now ();
    const double       dt  = std::max ((now - last_update_time_).seconds (), 0.0);
    last_update_time_      = now;

    if (!power_state_) {
        target_linear_speed_m_s_  = 0.0;
        target_yaw_speed_rad_s_   = 0.0;
        current_linear_speed_m_s_ = 0.0;
        current_yaw_speed_rad_s_  = 0.0;
        publish_joint_commands (0.0, 0.0);
        publish_command_velocity (0.0, 0.0);
        return;
    }

    if (auto_mode_state_) {
        const double auto_age_sec = std::max ((now - last_auto_command_time_).seconds (), 0.0);
        if (auto_age_sec <= auto_command_timeout_sec_) {
            target_linear_speed_m_s_ = auto_target_linear_speed_m_s_;
            target_yaw_speed_rad_s_  = auto_target_yaw_speed_rad_s_;
        } else {
            target_linear_speed_m_s_ = 0.0;
            target_yaw_speed_rad_s_  = 0.0;
        }
    }

    update_motion (dt);
    publish_joint_commands (current_linear_speed_m_s_, current_yaw_speed_rad_s_);
    publish_command_velocity (current_linear_speed_m_s_, current_yaw_speed_rad_s_);
}

void pbl_control::joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg) {
    const rclcpp::Time now = this->now ();
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

    if (button_or_zero (kPowerOnButtonIndex) == 1) {
        power_state_ = true;
    }
    if (button_or_zero (kPowerOffButtonIndex) == 1) {
        power_state_ = false;
    }
    if (button_or_zero (kAutoModeOnButtonIndex) == 1) {
        auto_mode_state_ = true;
    }
    if (button_or_zero (kAutoModeOffButtonIndex) == 1) {
        auto_mode_state_         = false;
        target_linear_speed_m_s_ = 0.0;
        target_yaw_speed_rad_s_  = 0.0;
    }

    if (!power_state_) {
        auto_mode_state_          = false;
        target_linear_speed_m_s_  = 0.0;
        target_yaw_speed_rad_s_   = 0.0;
        current_linear_speed_m_s_ = 0.0;
        current_yaw_speed_rad_s_  = 0.0;
        return;
    }

    if (!auto_mode_state_) {
        target_linear_speed_m_s_ = raw_forward_axis * max_linear_speed_m_s_;
        target_yaw_speed_rad_s_  = raw_turn_axis * max_yaw_speed_rad_s_;
    }
}

}  // namespace pbl_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (pbl_control::pbl_control)
