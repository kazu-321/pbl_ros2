#ifndef PBL_SIMULATION__PBL_JOINT_STATE_SIMULATOR_HPP_
#define PBL_SIMULATION__PBL_JOINT_STATE_SIMULATOR_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <string>

namespace pbl_simulation {

class pbl_joint_state_simulator : public rclcpp::Node {
   public:
    explicit pbl_joint_state_simulator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

   private:
    struct WheelCommand {
        double      right_wheel_speed_rad_s = 0.0;
        double      left_wheel_speed_rad_s  = 0.0;
        rclcpp::Time stamp                  = rclcpp::Time(0, 0, RCL_ROS_TIME);
        bool        valid                  = false;
    };

    void joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void timer_callback();

    void store_joint_command(const sensor_msgs::msg::JointState & msg, const rclcpp::Time & stamp);
    void store_cmd_vel_command(
        const geometry_msgs::msg::Twist & msg, const rclcpp::Time & stamp, WheelCommand & target_command);
    bool choose_active_command(const rclcpp::Time & now, WheelCommand & out_command) const;
    void advance_state(double dt, const WheelCommand & target_command);
    void publish_joint_state(const rclcpp::Time & stamp) const;

    bool extract_joint_speed(
        const sensor_msgs::msg::JointState & msg, const std::string & joint_name, double & speed_rad_s) const;
    double clamp_speed(double value) const;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr    cmd_vel_nav_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr    cmd_vel_smoothed_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr    cmd_vel_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr    joint_state_pub_;
    rclcpp::TimerBase::SharedPtr                                   timer_;

    double wheel_radius_;
    double wheel_separation_;
    double max_wheel_speed_rad_s_;
    double wheel_acceleration_rad_s2_;
    double publish_rate_hz_;
    double command_timeout_sec_;
    std::string joint_command_topic_;
    std::string cmd_vel_nav_topic_;
    std::string cmd_vel_smoothed_topic_;
    std::string cmd_vel_topic_;
    std::string joint_state_topic_;

    rclcpp::Time last_update_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    bool         has_last_update_time_ = false;

    WheelCommand joint_command_;
    WheelCommand cmd_vel_nav_command_;
    WheelCommand cmd_vel_smoothed_command_;
    WheelCommand cmd_vel_command_;

    double right_wheel_position_rad_ = 0.0;
    double left_wheel_position_rad_  = 0.0;
    double right_wheel_velocity_rad_s_ = 0.0;
    double left_wheel_velocity_rad_s_  = 0.0;
};

}  // namespace pbl_simulation

#endif  // PBL_SIMULATION__PBL_JOINT_STATE_SIMULATOR_HPP_
