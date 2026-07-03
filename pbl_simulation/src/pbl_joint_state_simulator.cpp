#include "pbl_simulation/pbl_joint_state_simulator.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <iterator>

namespace pbl_simulation {
namespace {
constexpr const char *kRightWheelJointName = "wheel_right";
constexpr const char *kLeftWheelJointName = "wheel_left";

bool has_stamp(const builtin_interfaces::msg::Time & stamp) {
    return stamp.sec != 0 || stamp.nanosec != 0;
}
}  // namespace

pbl_joint_state_simulator::pbl_joint_state_simulator(const rclcpp::NodeOptions & options)
    : Node("pbl_joint_state_simulator", options),
      wheel_radius_(this->declare_parameter<double>("wheel_radius", 0.05)),
      wheel_separation_(this->declare_parameter<double>("wheel_separation", 0.46)),
      max_wheel_speed_rad_s_(this->declare_parameter<double>("max_wheel_speed_rad_s", 30.0)),
      wheel_acceleration_rad_s2_(this->declare_parameter<double>("wheel_acceleration_rad_s2", 30.0)),
      publish_rate_hz_(this->declare_parameter<double>("publish_rate_hz", 100.0)),
      command_timeout_sec_(this->declare_parameter<double>("command_timeout_sec", 0.5)),
      joint_command_topic_(this->declare_parameter<std::string>("joint_command_topic", "/joint_commands")),
      cmd_vel_nav_topic_(this->declare_parameter<std::string>("cmd_vel_nav_topic", "/cmd_vel_nav")),
      cmd_vel_smoothed_topic_(this->declare_parameter<std::string>("cmd_vel_smoothed_topic", "/cmd_vel_smoothed")),
      cmd_vel_topic_(this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel")),
      joint_state_topic_(this->declare_parameter<std::string>("joint_state_topic", "/joint_states")) {
    if (wheel_radius_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "wheel_radius must be positive. Falling back to 0.05 m.");
        wheel_radius_ = 0.05;
    }
    if (wheel_separation_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "wheel_separation must be positive. Falling back to 0.46 m.");
        wheel_separation_ = 0.46;
    }
    if (max_wheel_speed_rad_s_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "max_wheel_speed_rad_s must be positive. Falling back to 30 rad/s.");
        max_wheel_speed_rad_s_ = 30.0;
    }
    if (wheel_acceleration_rad_s2_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "wheel_acceleration_rad_s2 must be positive. Falling back to 30 rad/s^2.");
        wheel_acceleration_rad_s2_ = 30.0;
    }
    if (publish_rate_hz_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "publish_rate_hz must be positive. Falling back to 100 Hz.");
        publish_rate_hz_ = 100.0;
    }
    if (command_timeout_sec_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "command_timeout_sec must be positive. Falling back to 0.5 s.");
        command_timeout_sec_ = 0.5;
    }

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, rclcpp::QoS(10));
    joint_command_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        joint_command_topic_, rclcpp::SensorDataQoS(),
        std::bind(&pbl_joint_state_simulator::joint_command_callback, this, std::placeholders::_1));
    cmd_vel_nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_nav_topic_, rclcpp::QoS(10),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            this->store_cmd_vel_command(*msg, this->now(), this->cmd_vel_nav_command_);
        });
    cmd_vel_smoothed_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_smoothed_topic_, rclcpp::QoS(10),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            this->store_cmd_vel_command(*msg, this->now(), this->cmd_vel_smoothed_command_);
        });
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, rclcpp::QoS(10),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            this->store_cmd_vel_command(*msg, this->now(), this->cmd_vel_command_);
        });

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_));
    timer_ = this->create_wall_timer(period, std::bind(&pbl_joint_state_simulator::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "pbl_joint_state_simulator initialized");
    RCLCPP_INFO(this->get_logger(), "wheel_radius: %.3f", wheel_radius_);
    RCLCPP_INFO(this->get_logger(), "wheel_separation: %.3f", wheel_separation_);
    RCLCPP_INFO(this->get_logger(), "max_wheel_speed_rad_s: %.3f", max_wheel_speed_rad_s_);
    RCLCPP_INFO(this->get_logger(), "wheel_acceleration_rad_s2: %.3f", wheel_acceleration_rad_s2_);
    RCLCPP_INFO(this->get_logger(), "publish_rate_hz: %.3f", publish_rate_hz_);
    RCLCPP_INFO(this->get_logger(), "command_timeout_sec: %.3f", command_timeout_sec_);
    RCLCPP_INFO(this->get_logger(), "joint_command_topic: %s", joint_command_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "cmd_vel topics: %s, %s, %s", cmd_vel_nav_topic_.c_str(), cmd_vel_smoothed_topic_.c_str(), cmd_vel_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "joint_state_topic: %s", joint_state_topic_.c_str());
}

void pbl_joint_state_simulator::joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    const rclcpp::Time stamp = has_stamp(msg->header.stamp) ? rclcpp::Time(msg->header.stamp) : this->now();
    store_joint_command(*msg, stamp);
}

void pbl_joint_state_simulator::store_joint_command(const sensor_msgs::msg::JointState & msg, const rclcpp::Time & stamp) {
    double right_speed_rad_s = 0.0;
    double left_speed_rad_s = 0.0;
    if (!extract_joint_speed(msg, kRightWheelJointName, right_speed_rad_s) ||
        !extract_joint_speed(msg, kLeftWheelJointName, left_speed_rad_s)) {
        if (msg.velocity.size() < 2) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 3000,
                "joint_commands did not contain wheel_right/wheel_left velocity entries.");
            return;
        }
        right_speed_rad_s = msg.velocity[0];
        left_speed_rad_s = msg.velocity[1];
    }

    joint_command_.right_wheel_speed_rad_s = clamp_speed(right_speed_rad_s);
    joint_command_.left_wheel_speed_rad_s = clamp_speed(left_speed_rad_s);
    joint_command_.stamp = stamp;
    joint_command_.valid = true;
}

void pbl_joint_state_simulator::store_cmd_vel_command(
    const geometry_msgs::msg::Twist & msg, const rclcpp::Time & stamp, WheelCommand & target_command) {
    const double linear_speed_m_s = msg.linear.x;
    const double yaw_speed_rad_s = msg.angular.z;
    const double right_speed_rad_s = clamp_speed(
        (linear_speed_m_s + yaw_speed_rad_s * wheel_separation_ * 0.5) / wheel_radius_);
    const double left_speed_rad_s = clamp_speed(
        (linear_speed_m_s - yaw_speed_rad_s * wheel_separation_ * 0.5) / wheel_radius_);

    target_command.right_wheel_speed_rad_s = right_speed_rad_s;
    target_command.left_wheel_speed_rad_s = left_speed_rad_s;
    target_command.stamp = stamp;
    target_command.valid = true;
}

bool pbl_joint_state_simulator::choose_active_command(const rclcpp::Time & now, WheelCommand & out_command) const {
    const auto is_fresh = [this, &now](const WheelCommand & command) {
        return command.valid && ((now - command.stamp).seconds() <= command_timeout_sec_);
    };

    if (is_fresh(joint_command_)) {
        out_command = joint_command_;
        return true;
    }
    if (is_fresh(cmd_vel_nav_command_)) {
        out_command = cmd_vel_nav_command_;
        return true;
    }
    if (is_fresh(cmd_vel_smoothed_command_)) {
        out_command = cmd_vel_smoothed_command_;
        return true;
    }
    if (is_fresh(cmd_vel_command_)) {
        out_command = cmd_vel_command_;
        return true;
    }

    out_command.right_wheel_speed_rad_s = 0.0;
    out_command.left_wheel_speed_rad_s = 0.0;
    out_command.stamp = now;
    out_command.valid = true;
    return false;
}

void pbl_joint_state_simulator::advance_state(double dt, const WheelCommand & target_command) {
    const double right_prev_velocity = right_wheel_velocity_rad_s_;
    const double left_prev_velocity = left_wheel_velocity_rad_s_;
    const double max_delta = wheel_acceleration_rad_s2_ * dt;

    right_wheel_velocity_rad_s_ += std::clamp(
        target_command.right_wheel_speed_rad_s - right_wheel_velocity_rad_s_, -max_delta, max_delta);
    left_wheel_velocity_rad_s_ += std::clamp(
        target_command.left_wheel_speed_rad_s - left_wheel_velocity_rad_s_, -max_delta, max_delta);

    right_wheel_velocity_rad_s_ = clamp_speed(right_wheel_velocity_rad_s_);
    left_wheel_velocity_rad_s_ = clamp_speed(left_wheel_velocity_rad_s_);

    right_wheel_position_rad_ += 0.5 * (right_prev_velocity + right_wheel_velocity_rad_s_) * dt;
    left_wheel_position_rad_ += 0.5 * (left_prev_velocity + left_wheel_velocity_rad_s_) * dt;
}

void pbl_joint_state_simulator::publish_joint_state(const rclcpp::Time & stamp) const {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "base_link";
    msg.name = {kRightWheelJointName, kLeftWheelJointName};
    msg.position = {right_wheel_position_rad_, left_wheel_position_rad_};
    msg.velocity = {right_wheel_velocity_rad_s_, left_wheel_velocity_rad_s_};
    msg.effort = {0.0, 0.0};
    joint_state_pub_->publish(msg);
}

void pbl_joint_state_simulator::timer_callback() {
    const rclcpp::Time now = this->now();
    if (!has_last_update_time_) {
        last_update_time_ = now;
        has_last_update_time_ = true;
        publish_joint_state(now);
        return;
    }

    const double dt = std::max((now - last_update_time_).seconds(), 0.0);
    last_update_time_ = now;
    if (dt <= 0.0) {
        publish_joint_state(now);
        return;
    }

    WheelCommand active_command;
    choose_active_command(now, active_command);
    advance_state(dt, active_command);
    publish_joint_state(now);
}

bool pbl_joint_state_simulator::extract_joint_speed(
    const sensor_msgs::msg::JointState & msg, const std::string & joint_name, double & speed_rad_s) const {
    const auto it = std::find(msg.name.begin(), msg.name.end(), joint_name);
    if (it == msg.name.end()) {
        return false;
    }

    const std::size_t index = static_cast<std::size_t>(std::distance(msg.name.begin(), it));
    if (index >= msg.velocity.size()) {
        return false;
    }

    speed_rad_s = msg.velocity[index];
    return true;
}

double pbl_joint_state_simulator::clamp_speed(double value) const {
    return std::clamp(value, -max_wheel_speed_rad_s_, max_wheel_speed_rad_s_);
}

}  // namespace pbl_simulation

RCLCPP_COMPONENTS_REGISTER_NODE(pbl_simulation::pbl_joint_state_simulator)
