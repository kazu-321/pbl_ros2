#ifndef PBL_SIMULATION__PBL_LIDAR_SIMULATOR_HPP_
#define PBL_SIMULATION__PBL_LIDAR_SIMULATOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <string>

namespace pbl_simulation {

class pbl_lidar_simulator : public rclcpp::Node {
   public:
    explicit pbl_lidar_simulator (const rclcpp::NodeOptions &options = rclcpp::NodeOptions ());

   private:
    void clicked_point_callback (const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void timer_callback ();
    void publish_cloud (const rclcpp::Time &stamp);

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr       cloud_pub_;
    rclcpp::TimerBase::SharedPtr                                      timer_;

    tf2_ros::Buffer            tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string clicked_point_topic_;
    std::string output_topic_;
    std::string target_frame_id_;
    double      publish_rate_hz_;
    double      obstacle_radius_m_;
    double      obstacle_height_m_;
    double      point_spacing_m_;
    double      obstacle_timeout_sec_;

    geometry_msgs::msg::PointStamped obstacle_center_;
    bool                             has_obstacle_ = false;
    rclcpp::Time                     obstacle_received_time_;
};

}  // namespace pbl_simulation

#endif  // PBL_SIMULATION__PBL_LIDAR_SIMULATOR_HPP_
