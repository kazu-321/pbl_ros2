#include "pbl_simulation/pbl_lidar_simulator.hpp"

#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/exceptions.h"

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

namespace pbl_simulation {
namespace {
constexpr double kPi = 3.14159265358979323846;
}

pbl_lidar_simulator::pbl_lidar_simulator (const rclcpp::NodeOptions &options)
    : Node ("pbl_lidar_simulator", options),
      tf_buffer_ (this->get_clock ()),
      tf_listener_ (tf_buffer_, this, true),
      clicked_point_topic_ (this->declare_parameter<std::string> ("clicked_point_topic", "/clicked_point")),
      output_topic_ (this->declare_parameter<std::string> ("output_topic", "/unilidar/cloud_filtered")),
      target_frame_id_ (this->declare_parameter<std::string> ("target_frame_id", "base_link")),
      publish_rate_hz_ (this->declare_parameter<double> ("publish_rate_hz", 20.0)),
      obstacle_radius_m_ (this->declare_parameter<double> ("obstacle_radius_m", 0.25)),
      obstacle_height_m_ (this->declare_parameter<double> ("obstacle_height_m", 1.7)),
      point_spacing_m_ (this->declare_parameter<double> ("point_spacing_m", 0.08)),
      obstacle_timeout_sec_ (this->declare_parameter<double> ("obstacle_timeout_sec", 0.0)),
      obstacle_received_time_ (this->now ()) {
    publish_rate_hz_   = std::max (publish_rate_hz_, 1.0);
    obstacle_radius_m_ = std::max (obstacle_radius_m_, 0.05);
    obstacle_height_m_ = std::max (obstacle_height_m_, 0.2);
    point_spacing_m_   = std::max (point_spacing_m_, 0.02);

    cloud_pub_         = this->create_publisher<sensor_msgs::msg::PointCloud2> (output_topic_, rclcpp::SensorDataQoS ());
    clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped> (clicked_point_topic_, rclcpp::QoS (10), std::bind (&pbl_lidar_simulator::clicked_point_callback, this, std::placeholders::_1));

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds> (std::chrono::duration<double> (1.0 / publish_rate_hz_));
    timer_            = this->create_wall_timer (period, std::bind (&pbl_lidar_simulator::timer_callback, this));

    RCLCPP_INFO (this->get_logger (), "pbl_lidar_simulator initialized");
    RCLCPP_INFO (this->get_logger (), "clicked_point_topic: %s", clicked_point_topic_.c_str ());
    RCLCPP_INFO (this->get_logger (), "output_topic: %s", output_topic_.c_str ());
    RCLCPP_INFO (this->get_logger (), "target_frame_id: %s", target_frame_id_.c_str ());
    RCLCPP_INFO (this->get_logger (), "obstacle: radius=%.2f m height=%.2f m", obstacle_radius_m_, obstacle_height_m_);
}

void pbl_lidar_simulator::clicked_point_callback (const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    if (msg->header.frame_id.empty ()) {
        RCLCPP_WARN (this->get_logger (), "Ignoring clicked point without a frame_id");
        return;
    }

    obstacle_center_        = *msg;
    obstacle_received_time_ = this->now ();
    has_obstacle_           = true;
    RCLCPP_INFO (this->get_logger (), "Placed simulated obstacle at (%.2f, %.2f, %.2f) in frame '%s'", msg->point.x, msg->point.y, msg->point.z, msg->header.frame_id.c_str ());
}

void pbl_lidar_simulator::timer_callback () {
    publish_cloud (this->now ());
}

void pbl_lidar_simulator::publish_cloud (const rclcpp::Time &stamp) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp    = stamp;
    cloud.header.frame_id = target_frame_id_;

    std::vector<geometry_msgs::msg::Point> points;
    if (has_obstacle_ && (obstacle_timeout_sec_ <= 0.0 || (stamp - obstacle_received_time_).seconds () <= obstacle_timeout_sec_)) {
        geometry_msgs::msg::PointStamped center_in_target;
        bool                             center_valid = false;
        try {
            if (obstacle_center_.header.frame_id == target_frame_id_) {
                center_in_target = obstacle_center_;
            } else {
                const auto transform = tf_buffer_.lookupTransform (target_frame_id_, obstacle_center_.header.frame_id, tf2::TimePointZero, tf2::durationFromSec (0.1));
                tf2::doTransform (obstacle_center_, center_in_target, transform);
            }
            center_valid = true;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE (this->get_logger (), *this->get_clock (), 3000, "Could not transform clicked point to '%s': %s", target_frame_id_.c_str (), ex.what ());
        }

        if (center_valid) {
            const int radial_steps = std::max (8, static_cast<int> (std::ceil (2.0 * kPi * obstacle_radius_m_ / point_spacing_m_)));
            const int height_steps = std::max (2, static_cast<int> (std::ceil (obstacle_height_m_ / point_spacing_m_)));
            points.reserve (static_cast<std::size_t> (radial_steps * height_steps));
            for (int iz = 0; iz <= height_steps; ++iz) {
                const double z = std::min (obstacle_height_m_, static_cast<double> (iz) * obstacle_height_m_ / height_steps);
                for (int ia = 0; ia < radial_steps; ++ia) {
                    const double              angle = 2.0 * kPi * static_cast<double> (ia) / radial_steps;
                    geometry_msgs::msg::Point point;
                    point.x = center_in_target.point.x + obstacle_radius_m_ * std::cos (angle);
                    point.y = center_in_target.point.y + obstacle_radius_m_ * std::sin (angle);
                    point.z = center_in_target.point.z + z;
                    points.push_back (point);
                }
            }
        }
    }

    cloud.height       = 1;
    cloud.width        = static_cast<uint32_t> (points.size ());
    cloud.is_bigendian = false;
    cloud.is_dense     = true;
    sensor_msgs::PointCloud2Modifier modifier (cloud);
    modifier.setPointCloud2FieldsByString (1, "xyz");
    modifier.resize (points.size ());

    sensor_msgs::PointCloud2Iterator<float> x_it (cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y_it (cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z_it (cloud, "z");
    for (const auto &point : points) {
        *x_it = static_cast<float> (point.x);
        *y_it = static_cast<float> (point.y);
        *z_it = static_cast<float> (point.z);
        ++x_it;
        ++y_it;
        ++z_it;
    }
    cloud_pub_->publish (cloud);
}

}  // namespace pbl_simulation

RCLCPP_COMPONENTS_REGISTER_NODE (pbl_simulation::pbl_lidar_simulator)
