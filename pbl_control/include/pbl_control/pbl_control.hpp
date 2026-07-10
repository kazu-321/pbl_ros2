#ifndef __PBL_CONTROL_HPP__
#define __PBL_CONTROL_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/utils.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cstddef>
#include <string>

namespace pbl_control {
class pbl_control : public rclcpp::Node {
   public:
    explicit pbl_control (const rclcpp::NodeOptions &node_options);

   private:
    static constexpr std::size_t kLeftStickHorizontalAxis = 0;
    static constexpr std::size_t kLeftStickVerticalAxis   = 1;
    static constexpr std::size_t kPowerOnButtonIndex      = 1;
    static constexpr std::size_t kPowerOffButtonIndex     = 0;
    static constexpr std::size_t kAutoModeOffButtonIndex  = 2;
    static constexpr std::size_t kAutoModeOnButtonIndex   = 3;
    static constexpr const char *kRightWheelJointName     = "wheel_right";
    static constexpr const char *kLeftWheelJointName      = "wheel_left";

    double       chassis_max_wheel_speed_rad_s_;
    double       chassis_acceleration_rate_s_;
    double       wheel_radius_;
    double       wheel_separation_;
    double       wheel_axle_x_;
    double       configured_max_linear_speed_m_s_;
    double       configured_max_yaw_speed_rad_s_;
    double       max_linear_speed_m_s_;
    double       max_yaw_speed_rad_s_;
    double       current_linear_speed_m_s_;
    double       current_yaw_speed_rad_s_;
    double       max_linear_acceleration_m_s2_;
    double       max_yaw_acceleration_rad_s2_;
    bool         power_state_;
    bool         auto_mode_state_;
    rclcpp::Time last_joy_time_;
    rclcpp::Time last_update_time_;
    rclcpp::Time last_auto_command_time_;
    double       target_linear_speed_m_s_;
    double       target_yaw_speed_rad_s_;
    double       auto_target_linear_speed_m_s_;
    double       auto_target_yaw_speed_rad_s_;
    double       control_rate_hz_;
    double       auto_command_timeout_sec_;

    void joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg);
    void cmd_vel_smoothed_callback (const geometry_msgs::msg::Twist::SharedPtr msg);
    void control_timer_callback ();
    void update_motion (double dt);
    void publish_power_state ();
    void publish_auto_state ();
    void publish_joint_commands (double linear_speed_m_s, double yaw_speed_rad_s);
    void publish_command_velocity (double linear_speed_m_s, double yaw_speed_rad_s);
    void publish_state_topics ();

    static double apply_deadzone (double value, double deadzone);
    static double normalize_pair (double value_a, double value_b, double value);

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr     joint_command_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr              power_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr              auto_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr         joy_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr     cmd_vel_smoothed_sub_;
    rclcpp::TimerBase::SharedPtr                                   control_timer_;
    rclcpp::TimerBase::SharedPtr                                   state_timer_;
};
}  // namespace pbl_control

#endif  // __PBL_CONTROL_HPP__
