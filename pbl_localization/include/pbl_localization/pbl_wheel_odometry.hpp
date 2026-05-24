#ifndef PBL_WHEEL_ODOMETRY__PBL_WHEEL_ODOMETRY_HPP_
#define PBL_WHEEL_ODOMETRY__PBL_WHEEL_ODOMETRY_HPP_

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <memory>
#include <string>

namespace pbl_wheel_odometry {

class pbl_wheel_odometry : public rclcpp::Node {
   public:
    explicit pbl_wheel_odometry (const rclcpp::NodeOptions &options = rclcpp::NodeOptions ());

   private:
    void joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg);
    void publish_odometry (const rclcpp::Time &stamp, double linear_speed_m_s, double yaw_speed_rad_s);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr         odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster>                tf_broadcaster_;

    std::string odom_frame_id_;
    std::string base_frame_id_;
    double      wheel_radius_;
    double      wheel_separation_;
    bool        publish_tf_;

    bool         odom_initialized_ = false;
    rclcpp::Time last_joint_state_time_;
    double       odom_x_                        = 0.0;
    double       odom_y_                        = 0.0;
    double       odom_yaw_                      = 0.0;
    double       last_left_wheel_position_rad_  = 0.0;
    double       last_right_wheel_position_rad_ = 0.0;
};

}  // namespace pbl_wheel_odometry

#endif  // PBL_WHEEL_ODOMETRY__PBL_WHEEL_ODOMETRY_HPP_
