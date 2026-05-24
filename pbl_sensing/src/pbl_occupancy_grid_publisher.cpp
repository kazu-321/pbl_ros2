#include "pbl_sensing/pbl_occupancy_grid_publisher.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace pbl_sensing
{
namespace
{
constexpr int32_t kDefaultOccupiedCountFullScore = 10;
constexpr int kOccupiedValueMax = 100;

int count_to_occupancy(int count, int full_score)
{
  if (count <= 0) {
    return 0;
  }
  if (full_score <= 0) {
    return kOccupiedValueMax;
  }
  const int scaled = (count * kOccupiedValueMax) / full_score;
  return std::min(kOccupiedValueMax, std::max(1, scaled));
}

int8_t to_grid_value(int value)
{
  return static_cast<int8_t>(std::clamp(value, -1, 100));
}

bool has_field(const sensor_msgs::msg::PointCloud2 & msg, const std::string & field_name)
{
  return std::any_of(
    msg.fields.begin(), msg.fields.end(),
    [&field_name](const sensor_msgs::msg::PointField & field) {return field.name == field_name;});
}
}  // namespace

pbl_occupancy_grid_publisher::pbl_occupancy_grid_publisher(
  const rclcpp::NodeOptions & options)
: Node("pbl_occupancy_grid_publisher", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_, this, true),
  map_topic_(this->declare_parameter<std::string>("map_topic", "/map_3d")),
  cloud_topic_(this->declare_parameter<std::string>("cloud_topic", "/unilidar/cloud_filtered")),
  map_frame_id_(this->declare_parameter<std::string>("map_frame_id", "map")),
  z_min_(this->declare_parameter<double>("z_min", -0.2)),
  z_max_(this->declare_parameter<double>("z_max", 0.5)),
  grid_resolution_(this->declare_parameter<double>("grid_resolution", 0.05)),
  occupied_count_full_score_(
    static_cast<int>(this->declare_parameter<int32_t>(
      "occupied_count_full_score", kDefaultOccupiedCountFullScore))),
  transform_timeout_sec_(this->declare_parameter<double>("transform_timeout_sec", 0.1)) {
  tf_buffer_.setUsingDedicatedThread(true);

  occupancy_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "occupancy_grid", rclcpp::QoS(1).reliable().transient_local());
  used_map_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "used_map_cloud", rclcpp::QoS(1).reliable().transient_local());

  const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
  const auto cloud_qos = rclcpp::QoS(10);
  map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    map_topic_, map_qos,
    std::bind(&pbl_occupancy_grid_publisher::map_callback, this, std::placeholders::_1));
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, cloud_qos,
    std::bind(&pbl_occupancy_grid_publisher::cloud_callback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&pbl_occupancy_grid_publisher::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "pbl_occupancy_grid_publisher initialized");
  RCLCPP_INFO(this->get_logger(), "map_topic: %s", map_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "cloud_topic: %s", cloud_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "map_frame_id: %s", map_frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "z_min: %.3f", z_min_);
  RCLCPP_INFO(this->get_logger(), "z_max: %.3f", z_max_);
  RCLCPP_INFO(this->get_logger(), "grid_resolution: %.3f", grid_resolution_);
  RCLCPP_INFO(this->get_logger(), "occupied_count_full_score: %d", occupied_count_full_score_);
}

void pbl_occupancy_grid_publisher::map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (map_initialized_) {
    return;
  }

  if (!has_required_fields(*msg)) {
    RCLCPP_WARN(this->get_logger(), "Map cloud does not have x/y/z fields.");
    return;
  }

  sensor_msgs::msg::PointCloud2 map_in_frame;
  if (!transform_pointcloud(*msg, map_in_frame, map_frame_id_, rclcpp::Time(msg->header.stamp))) {
    return;
  }

  initialize_grid_from_map(map_in_frame);
  map_initialized_ = true;
  used_map_cloud_ = build_used_map_cloud(map_in_frame);
  build_static_occupancy_from_map(map_in_frame);
  used_map_cloud_pub_->publish(used_map_cloud_);
  occupancy_pub_->publish(occupancy_grid_);

  RCLCPP_INFO(
    this->get_logger(), "Initialized occupancy grid from %u x %u cells",
    occupancy_grid_.info.width, occupancy_grid_.info.height);
}

void pbl_occupancy_grid_publisher::cloud_callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!map_initialized_) {
    return;
  }

  if (!has_required_fields(*msg)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 3000,
      "Filtered cloud does not have x/y/z fields.");
    return;
  }

  sensor_msgs::msg::PointCloud2 cloud_in_map;
  if (!transform_pointcloud(*msg, cloud_in_map, map_frame_id_, rclcpp::Time(msg->header.stamp))) {
    return;
  }

  append_cloud_buffer(cloud_in_map);
  latest_cloud_ready_ = true;
}

void pbl_occupancy_grid_publisher::timer_callback()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!map_initialized_ || !latest_cloud_ready_) {
    return;
  }

  const auto buffered_cloud = take_cloud_buffer();
  if (buffered_cloud.data.empty()) {
    return;
  }

  rebuild_occupancy_grid(buffered_cloud);
  occupancy_grid_.header.stamp = this->now();
  occupancy_pub_->publish(occupancy_grid_);
}

bool pbl_occupancy_grid_publisher::has_required_fields(
  const sensor_msgs::msg::PointCloud2 & msg) const
{
  return has_field(msg, "x") && has_field(msg, "y") && has_field(msg, "z");
}

bool pbl_occupancy_grid_publisher::transform_pointcloud(
  const sensor_msgs::msg::PointCloud2 & in, sensor_msgs::msg::PointCloud2 & out,
  const std::string & target_frame, const rclcpp::Time & stamp) const
{
  try {
    if (in.header.frame_id == target_frame) {
      out = in;
      return true;
    }

    const auto transform = tf_buffer_.lookupTransform(
      target_frame, in.header.frame_id, stamp, tf2::durationFromSec(transform_timeout_sec_));
    tf2::doTransform(in, out, transform);
    out.header.stamp = in.header.stamp;
    out.header.frame_id = target_frame;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 3000,
      "Failed to transform PointCloud2 from '%s' to '%s': %s",
      in.header.frame_id.c_str(), target_frame.c_str(), ex.what());
    return false;
  }
}

void pbl_occupancy_grid_publisher::initialize_grid_from_map(
  const sensor_msgs::msg::PointCloud2 & map_cloud)
{
  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();

  sensor_msgs::PointCloud2ConstIterator<float> x_it(map_cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_it(map_cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_it(map_cloud, "z");

  const std::size_t point_count = static_cast<std::size_t>(map_cloud.width) * map_cloud.height;
  for (std::size_t i = 0; i < point_count; ++i, ++x_it, ++y_it, ++z_it) {
    const double x = static_cast<double>(*x_it);
    const double y = static_cast<double>(*y_it);
    const double z = static_cast<double>(*z_it);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    min_x = std::min(min_x, x);
    max_x = std::max(max_x, x);
    min_y = std::min(min_y, y);
    max_y = std::max(max_y, y);
  }

  if (!std::isfinite(min_x) || !std::isfinite(min_y) || !std::isfinite(max_x) ||
    !std::isfinite(max_y))
  {
    RCLCPP_WARN(this->get_logger(), "Map cloud is empty or invalid.");
    return;
  }

  const double origin_x = std::floor(min_x / grid_resolution_) * grid_resolution_;
  const double origin_y = std::floor(min_y / grid_resolution_) * grid_resolution_;
  const std::size_t width = static_cast<std::size_t>(
    std::ceil((max_x - origin_x) / grid_resolution_)) + 1;
  const std::size_t height = static_cast<std::size_t>(
    std::ceil((max_y - origin_y) / grid_resolution_)) + 1;

  occupancy_grid_.header.frame_id = map_frame_id_;
  occupancy_grid_.info.resolution = static_cast<float>(grid_resolution_);
  occupancy_grid_.info.width = static_cast<uint32_t>(width);
  occupancy_grid_.info.height = static_cast<uint32_t>(height);
  occupancy_grid_.info.origin.position.x = origin_x;
  occupancy_grid_.info.origin.position.y = origin_y;
  occupancy_grid_.info.origin.position.z = 0.0;
  occupancy_grid_.info.origin.orientation.w = 1.0;

  static_occupancy_.assign(width * height, -1);
  map_cell_present_.assign(width * height, 0);

  occupancy_grid_.data.assign(width * height, -1);
  build_static_occupancy_from_map(map_cloud);
}

void pbl_occupancy_grid_publisher::build_static_occupancy_from_map(
  const sensor_msgs::msg::PointCloud2 & map_cloud)
{
  if (occupancy_grid_.info.width == 0 || occupancy_grid_.info.height == 0) {
    return;
  }

  std::fill(static_occupancy_.begin(), static_occupancy_.end(), -1);
  std::fill(map_cell_present_.begin(), map_cell_present_.end(), 0);

  std::vector<int> cell_counts(static_cast<std::size_t>(occupancy_grid_.info.width) *
    static_cast<std::size_t>(occupancy_grid_.info.height), 0);
  sensor_msgs::PointCloud2ConstIterator<float> x_it(map_cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_it(map_cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_it(map_cloud, "z");
  const std::size_t point_count = static_cast<std::size_t>(map_cloud.width) * map_cloud.height;

  for (std::size_t i = 0; i < point_count; ++i, ++x_it, ++y_it, ++z_it) {
    const double x = static_cast<double>(*x_it);
    const double y = static_cast<double>(*y_it);
    const double z = static_cast<double>(*z_it);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }

    std::size_t x_idx = 0;
    std::size_t y_idx = 0;
    if (!world_to_cell(x, y, x_idx, y_idx)) {
      continue;
    }

    const std::size_t idx = cell_index(x_idx, y_idx);
    map_cell_present_[idx] = 1;
    if (z < z_min_ || z > z_max_) {
      continue;
    }
    ++cell_counts[idx];
  }

  for (std::size_t idx = 0; idx < cell_counts.size(); ++idx) {
    if (!map_cell_present_[idx]) {
      continue;
    }
    static_occupancy_[idx] = to_grid_value(
      count_to_occupancy(cell_counts[idx], occupied_count_full_score_));
  }
}

sensor_msgs::msg::PointCloud2 pbl_occupancy_grid_publisher::build_used_map_cloud(
  const sensor_msgs::msg::PointCloud2 & map_cloud) const
{
  sensor_msgs::msg::PointCloud2 used;
  used.header = map_cloud.header;
  used.height = 1;
  used.is_bigendian = map_cloud.is_bigendian;
  used.is_dense = false;
  used.fields = map_cloud.fields;
  used.point_step = map_cloud.point_step;
  used.data.reserve(map_cloud.data.size());

  sensor_msgs::PointCloud2ConstIterator<float> x_it(map_cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_it(map_cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_it(map_cloud, "z");
  const std::size_t point_count = static_cast<std::size_t>(map_cloud.width) * map_cloud.height;

  for (std::size_t i = 0; i < point_count; ++i, ++x_it, ++y_it, ++z_it) {
    const double x = static_cast<double>(*x_it);
    const double y = static_cast<double>(*y_it);
    const double z = static_cast<double>(*z_it);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }

    std::size_t x_idx = 0;
    std::size_t y_idx = 0;
    if (!world_to_cell(x, y, x_idx, y_idx)) {
      continue;
    }

    if (z < z_min_ || z > z_max_) {
      continue;
    }

    const std::size_t row = i / map_cloud.width;
    const std::size_t col = i % map_cloud.width;
    const std::size_t offset = row * map_cloud.row_step + col * map_cloud.point_step;
    used.data.insert(
      used.data.end(),
      map_cloud.data.begin() + static_cast<std::ptrdiff_t>(offset),
      map_cloud.data.begin() + static_cast<std::ptrdiff_t>(offset + map_cloud.point_step));
  }

  used.width = static_cast<uint32_t>(used.data.size() / used.point_step);
  used.row_step = used.width * used.point_step;
  used.header.frame_id = map_frame_id_;
  used.header.stamp = map_cloud.header.stamp;
  return used;
}

void pbl_occupancy_grid_publisher::append_cloud_buffer(
  const sensor_msgs::msg::PointCloud2 & cloud_in_map)
{
  if (cloud_buffer_.fields.empty()) {
    cloud_buffer_ = cloud_in_map;
    cloud_buffer_.height = 1;
    cloud_buffer_.row_step = cloud_in_map.point_step * cloud_in_map.width;
    cloud_buffer_.width = cloud_in_map.width;
    return;
  }

  if (cloud_buffer_.fields != cloud_in_map.fields || cloud_buffer_.point_step != cloud_in_map.point_step) {
    cloud_buffer_ = cloud_in_map;
    cloud_buffer_.height = 1;
    cloud_buffer_.row_step = cloud_in_map.point_step * cloud_in_map.width;
    cloud_buffer_.width = cloud_in_map.width;
    return;
  }

  cloud_buffer_.data.insert(
    cloud_buffer_.data.end(), cloud_in_map.data.begin(), cloud_in_map.data.end());
  cloud_buffer_.width += cloud_in_map.width;
  cloud_buffer_.row_step = cloud_buffer_.point_step * cloud_buffer_.width;
  cloud_buffer_.header.stamp = cloud_in_map.header.stamp;
  cloud_buffer_.header.frame_id = cloud_in_map.header.frame_id;
}

sensor_msgs::msg::PointCloud2 pbl_occupancy_grid_publisher::take_cloud_buffer()
{
  sensor_msgs::msg::PointCloud2 buffered = std::move(cloud_buffer_);
  cloud_buffer_ = sensor_msgs::msg::PointCloud2{};
  return buffered;
}

bool pbl_occupancy_grid_publisher::world_to_cell(
  double x, double y, std::size_t & x_idx, std::size_t & y_idx) const
{
  if (occupancy_grid_.info.width == 0 || occupancy_grid_.info.height == 0) {
    return false;
  }

  const double origin_x = occupancy_grid_.info.origin.position.x;
  const double origin_y = occupancy_grid_.info.origin.position.y;
  const double res = occupancy_grid_.info.resolution;

  if (x < origin_x || y < origin_y) {
    return false;
  }

  const double rel_x = (x - origin_x) / res;
  const double rel_y = (y - origin_y) / res;
  if (rel_x < 0.0 || rel_y < 0.0) {
    return false;
  }

  x_idx = static_cast<std::size_t>(std::floor(rel_x));
  y_idx = static_cast<std::size_t>(std::floor(rel_y));

  if (x_idx >= occupancy_grid_.info.width || y_idx >= occupancy_grid_.info.height) {
    return false;
  }
  return true;
}

std::size_t pbl_occupancy_grid_publisher::cell_index(std::size_t x_idx, std::size_t y_idx) const
{
  return y_idx * occupancy_grid_.info.width + x_idx;
}

void pbl_occupancy_grid_publisher::rebuild_occupancy_grid(
  const sensor_msgs::msg::PointCloud2 & cloud_in_map)
{
  const std::size_t total_cells = occupancy_grid_.info.width * occupancy_grid_.info.height;
  occupancy_grid_.data = static_occupancy_;

  std::vector<int> occupied_counts(total_cells, 0);

  sensor_msgs::PointCloud2ConstIterator<float> x_it(cloud_in_map, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_it(cloud_in_map, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_it(cloud_in_map, "z");

  const std::size_t point_count = static_cast<std::size_t>(cloud_in_map.width) * cloud_in_map.height;
  for (std::size_t i = 0; i < point_count; ++i, ++x_it, ++y_it, ++z_it) {
    const double x = static_cast<double>(*x_it);
    const double y = static_cast<double>(*y_it);
    const double z = static_cast<double>(*z_it);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }

    std::size_t x_idx = 0;
    std::size_t y_idx = 0;
    if (!world_to_cell(x, y, x_idx, y_idx)) {
      continue;
    }

    const std::size_t idx = cell_index(x_idx, y_idx);
    if (z >= z_min_ && z <= z_max_) {
      ++occupied_counts[idx];
    }
  }

  for (std::size_t idx = 0; idx < total_cells; ++idx) {
    if (!map_cell_present_[idx]) {
      continue;
    }
    const int dynamic_value = count_to_occupancy(occupied_counts[idx], occupied_count_full_score_);
    occupancy_grid_.data[idx] = to_grid_value(
      std::max<int>(static_cast<int>(occupancy_grid_.data[idx]), dynamic_value));
  }
}

}  // namespace pbl_sensing

RCLCPP_COMPONENTS_REGISTER_NODE(pbl_sensing::pbl_occupancy_grid_publisher)
