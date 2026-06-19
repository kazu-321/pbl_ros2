#include "pbl_sensing/pbl_pointcloud2_filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <chrono>
#include <limits>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/time.hpp"
#include "tf2/exceptions.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace pbl_sensing
{
namespace
{
bool has_field(const sensor_msgs::msg::PointCloud2 & msg, const std::string & field_name)
{
  return std::any_of(
    msg.fields.begin(), msg.fields.end(),
    [&field_name](const sensor_msgs::msg::PointField & field) {return field.name == field_name;});
}
}  // namespace

pbl_pointcloud2_filter::pbl_pointcloud2_filter(const rclcpp::NodeOptions & options)
: Node("pbl_pointcloud2_filter", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_, this, true),
  input_topic_(this->declare_parameter<std::string>("input_topic", "/unilidar/cloud")),
  output_topic_(this->declare_parameter<std::string>("output_topic", "/unilidar/cloud_filtered")),
  output_topic_transformed_(
    this->declare_parameter<std::string>(
      "output_topic_transformed", "/unilidar/cloud_filtered_transformed")),
  target_frame_id_(this->declare_parameter<std::string>("target_frame_id", "base_link")),
  transform_timeout_sec_(this->declare_parameter<double>("transform_timeout_sec", 0.1)) {
  tf_buffer_.setUsingDedicatedThread(true);

  const double body_center_x = this->declare_parameter<double>("body_box_center_x", 0.14);
  const double body_center_y = this->declare_parameter<double>("body_box_center_y", 0.0);
  const double body_center_z = this->declare_parameter<double>("body_box_center_z", 0.0575);
  const double body_size_x = this->declare_parameter<double>("body_box_size_x", 0.45);
  const double body_size_y = this->declare_parameter<double>("body_box_size_y", 0.45);
  const double body_size_z = this->declare_parameter<double>("body_box_size_z", 0.015);

  const double pc_center_x = this->declare_parameter<double>("pc_box_center_x", -0.04);
  const double pc_center_y = this->declare_parameter<double>("pc_box_center_y", 0.0);
  const double pc_center_z = this->declare_parameter<double>("pc_box_center_z", 0.315);
  const double pc_size_x = this->declare_parameter<double>("pc_box_size_x", 0.1);
  const double pc_size_y = this->declare_parameter<double>("pc_box_size_y", 0.45);
  const double pc_size_z = this->declare_parameter<double>("pc_box_size_z", 0.5);

  collision_boxes_.push_back(CollisionBox{
    "body",
    body_center_x,
    body_center_y,
    body_center_z,
    body_size_x,
    body_size_y,
    body_size_z});
  collision_boxes_.push_back(CollisionBox{
    "pc",
    pc_center_x,
    pc_center_y,
    pc_center_z,
    pc_size_x,
    pc_size_y,
    pc_size_z});

  const auto cloud_qos = rclcpp::QoS(10);
  filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    output_topic_, cloud_qos);
  filtered_cloud_transformed_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    output_topic_transformed_, cloud_qos);
  collision_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "collision_boxes", rclcpp::QoS(1).reliable());
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, cloud_qos,
    std::bind(&pbl_pointcloud2_filter::pointcloud_callback, this, std::placeholders::_1));
  collision_marker_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&pbl_pointcloud2_filter::publish_collision_markers, this));

  RCLCPP_INFO(this->get_logger(), "pbl_pointcloud2_filter initialized");
  RCLCPP_INFO(this->get_logger(), "input_topic: %s", input_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "output_topic: %s", output_topic_.c_str());
  RCLCPP_INFO(
    this->get_logger(), "output_topic_transformed: %s",
    output_topic_transformed_.c_str());
  RCLCPP_INFO(this->get_logger(), "target_frame_id: %s", target_frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "transform_timeout_sec: %.3f", transform_timeout_sec_);
  for (const auto & box : collision_boxes_) {
    RCLCPP_INFO(
      this->get_logger(),
      "collision_box[%s]: center=(%.3f, %.3f, %.3f) size=(%.3f, %.3f, %.3f)",
      box.name.c_str(), box.center_x, box.center_y, box.center_z,
      box.size_x, box.size_y, box.size_z);
  }

  publish_collision_markers();
}

void pbl_pointcloud2_filter::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!has_required_fields(*msg)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 3000,
      "PointCloud2 does not have x/y/z fields. Passing the cloud through unchanged.");
    filtered_cloud_pub_->publish(*msg);
    return;
  }

  const std::string original_frame_id = msg->header.frame_id;
  sensor_msgs::msg::PointCloud2 cloud_in_target;
  try {
    if (original_frame_id == target_frame_id_) {
      cloud_in_target = *msg;
    } else {
      const auto transform = tf_buffer_.lookupTransform(
        target_frame_id_, original_frame_id, tf2::TimePointZero,
        tf2::durationFromSec(transform_timeout_sec_));
      tf2::doTransform(*msg, cloud_in_target, transform);
    }
    cloud_in_target.header.stamp = msg->header.stamp;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 3000,
      "Failed to transform PointCloud2 from '%s' to '%s': %s",
      msg->header.frame_id.c_str(), target_frame_id_.c_str(), ex.what());
    return;
  }

  auto filtered_cloud = filter_self_points(cloud_in_target);
  filtered_cloud.header.stamp = msg->header.stamp;
  filtered_cloud_transformed_pub_->publish(filtered_cloud);

  if (original_frame_id != target_frame_id_) {
    try {
      const auto transform_back = tf_buffer_.lookupTransform(
        original_frame_id, target_frame_id_, tf2::TimePointZero,
        tf2::durationFromSec(transform_timeout_sec_));
      sensor_msgs::msg::PointCloud2 filtered_cloud_original;
      tf2::doTransform(filtered_cloud, filtered_cloud_original, transform_back);
      filtered_cloud_original.header.stamp = msg->header.stamp;
      filtered_cloud_pub_->publish(filtered_cloud_original);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Failed to transform filtered PointCloud2 back from '%s' to '%s': %s",
        target_frame_id_.c_str(), original_frame_id.c_str(), ex.what());
      return;
    }
  } else {
    filtered_cloud_pub_->publish(filtered_cloud);
  }
}

void pbl_pointcloud2_filter::publish_collision_markers()
{
  visualization_msgs::msg::MarkerArray marker_array;
  const auto now = this->now();

  auto make_corner = [](double x, double y, double z) {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
  };

  for (std::size_t i = 0; i < collision_boxes_.size(); ++i) {
    const auto & box = collision_boxes_[i];
    const double hx = box.size_x * 0.5;
    const double hy = box.size_y * 0.5;
    const double hz = box.size_z * 0.5;

    const std::array<geometry_msgs::msg::Point, 8> corners = {
      make_corner(box.center_x - hx, box.center_y - hy, box.center_z - hz),
      make_corner(box.center_x + hx, box.center_y - hy, box.center_z - hz),
      make_corner(box.center_x + hx, box.center_y + hy, box.center_z - hz),
      make_corner(box.center_x - hx, box.center_y + hy, box.center_z - hz),
      make_corner(box.center_x - hx, box.center_y - hy, box.center_z + hz),
      make_corner(box.center_x + hx, box.center_y - hy, box.center_z + hz),
      make_corner(box.center_x + hx, box.center_y + hy, box.center_z + hz),
      make_corner(box.center_x - hx, box.center_y + hy, box.center_z + hz),
    };

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = target_frame_id_;
    marker.header.stamp = now;
    marker.ns = "pbl_pointcloud2_filter";
    marker.id = static_cast<int32_t>(i);
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02;
    marker.color.a = 0.9f;
    marker.color.r = (box.name == "body") ? 0.1f : 0.9f;
    marker.color.g = (box.name == "body") ? 0.9f : 0.4f;
    marker.color.b = 0.1f;
    marker.lifetime = rclcpp::Duration(0, 500000000);  // 0.5 seconds

    const auto add_edge = [&](std::size_t a, std::size_t b) {
      marker.points.push_back(corners[a]);
      marker.points.push_back(corners[b]);
    };

    add_edge(0, 1);
    add_edge(1, 2);
    add_edge(2, 3);
    add_edge(3, 0);
    add_edge(4, 5);
    add_edge(5, 6);
    add_edge(6, 7);
    add_edge(7, 4);
    add_edge(0, 4);
    add_edge(1, 5);
    add_edge(2, 6);
    add_edge(3, 7);

    marker_array.markers.push_back(std::move(marker));
  }

  const auto clear_marker = [&](int32_t id) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = target_frame_id_;
    marker.header.stamp = now;
    marker.ns = "pbl_pointcloud2_filter";
    marker.id = id;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    marker_array.markers.push_back(std::move(marker));
  };

  if (collision_boxes_.empty()) {
    clear_marker(0);
    clear_marker(1);
  }

  collision_marker_pub_->publish(marker_array);
}

bool pbl_pointcloud2_filter::has_required_fields(const sensor_msgs::msg::PointCloud2 & msg) const
{
  return has_field(msg, "x") && has_field(msg, "y") && has_field(msg, "z");
}

bool pbl_pointcloud2_filter::is_inside_box(double x, double y, double z, const CollisionBox & box) const
{
  const double half_x = box.size_x * 0.5;
  const double half_y = box.size_y * 0.5;
  const double half_z = box.size_z * 0.5;

  return std::abs(x - box.center_x) <= half_x &&
         std::abs(y - box.center_y) <= half_y &&
         std::abs(z - box.center_z) <= half_z;
}

sensor_msgs::msg::PointCloud2 pbl_pointcloud2_filter::filter_self_points(
  const sensor_msgs::msg::PointCloud2 & cloud) const
{
  if (cloud.point_step == 0 || cloud.width == 0 || cloud.height == 0) {
    return cloud;
  }

  sensor_msgs::msg::PointCloud2 filtered;
  filtered.header = cloud.header;
  filtered.header.stamp = cloud.header.stamp;
  filtered.height = 1;
  filtered.is_bigendian = cloud.is_bigendian;
  filtered.is_dense = cloud.is_dense;
  filtered.fields = cloud.fields;
  filtered.point_step = cloud.point_step;
  filtered.is_dense = false;

  const std::size_t point_count = cloud.width * cloud.height;
  filtered.data.reserve(cloud.data.size());

  sensor_msgs::PointCloud2ConstIterator<float> x_it(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_it(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_it(cloud, "z");

  for (std::size_t i = 0; i < point_count; ++i, ++x_it, ++y_it, ++z_it) {
    const double x = static_cast<double>(*x_it);
    const double y = static_cast<double>(*y_it);
    const double z = static_cast<double>(*z_it);

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }

    bool inside = false;
    for (const auto & box : collision_boxes_) {
      if (is_inside_box(x, y, z, box)) {
        inside = true;
        break;
      }
    }
    if (inside) {
      continue;
    }

    const std::size_t row = i / cloud.width;
    const std::size_t col = i % cloud.width;
    const std::size_t offset = row * cloud.row_step + col * cloud.point_step;
    filtered.data.insert(
      filtered.data.end(),
      cloud.data.begin() + static_cast<std::ptrdiff_t>(offset),
      cloud.data.begin() + static_cast<std::ptrdiff_t>(offset + cloud.point_step));
  }

  filtered.width = static_cast<std::uint32_t>(filtered.data.size() / cloud.point_step);
  filtered.row_step = filtered.width * filtered.point_step;
  return filtered;
}

}  // namespace pbl_sensing

RCLCPP_COMPONENTS_REGISTER_NODE(pbl_sensing::pbl_pointcloud2_filter)
