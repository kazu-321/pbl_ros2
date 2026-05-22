#ifndef __PBL_CONTROL_HPP__
#define __PBL_CONTROL_HPP__

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

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
    static constexpr const char *kRightWheelJointName     = "joint0";
    static constexpr const char *kLeftWheelJointName      = "joint1";

    double chassis_max_wheel_speed_rad_s_;
    double chassis_acceleration_rate_s_;
    double wheel_radius_;
    double wheel_separation_;
    double wheel_axle_x_;
    double max_linear_speed_m_s_;
    double max_yaw_speed_rad_s_;
    double current_linear_speed_m_s_;
    double current_yaw_speed_rad_s_;
    double max_linear_acceleration_m_s2_;
    double max_yaw_acceleration_rad_s2_;
    bool   power_state_;
    bool   odom_initialized_;

    std::string odom_frame_id_;
    std::string base_frame_id_;

    rclcpp::Time last_joy_time_;
    rclcpp::Time last_joint_state_time_;
    double       odom_x_;
    double       odom_y_;
    double       odom_yaw_;
    double       last_left_wheel_position_rad_;
    double       last_right_wheel_position_rad_;

    void joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg);
    void joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg);
    void publish_power_state ();
    void publish_joint_commands (double linear_speed_m_s, double yaw_speed_rad_s);
    void publish_command_velocity (double linear_speed_m_s, double yaw_speed_rad_s);
    void publish_odometry (const rclcpp::Time &stamp, double linear_speed_m_s, double yaw_speed_rad_s);

    static double apply_deadzone (double value, double deadzone);
    static double normalize_pair (double value_a, double value_b, double value);

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_velocity_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr power_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
}  // namespace pbl_control

#endif  // __PBL_CONTROL_HPP__