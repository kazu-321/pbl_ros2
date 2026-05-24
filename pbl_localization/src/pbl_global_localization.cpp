#include "pbl_localization/pbl_global_localization.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "pcl/filters/voxel_grid.h"
#include "pcl/registration/ndt.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp_components/register_node_macro.hpp"

#include <sstream>

namespace pbl_localization {

pbl_global_localization::pbl_global_localization(const rclcpp::NodeOptions & options)
: Node("pbl_global_localization", options),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
  map_cloud_(std::make_shared<pcl::PointCloud<PointT>>()),
  map_frame_id_(this->declare_parameter<std::string>("map_frame_id", "map")),
  odom_frame_id_(this->declare_parameter<std::string>("odom_frame_id", "odom")),
  base_frame_id_(this->declare_parameter<std::string>("base_frame_id", "base_link")),
  map_cloud_topic_(this->declare_parameter<std::string>("map_cloud_topic", "/map_3d")),
  source_cloud_topic_(this->declare_parameter<std::string>("source_cloud_topic", "/lio/cloud_registered")),
  initial_pose_topic_(this->declare_parameter<std::string>("initial_pose_topic", "/initialpose")),
  source_buffer_duration_sec_(this->declare_parameter<double>("source_buffer_duration_sec", 1.0)),
  alignment_period_sec_(this->declare_parameter<double>("alignment_period_sec", 1.0)),
  ndt_resolution_(this->declare_parameter<double>("ndt_resolution", 1.0)),
  ndt_step_size_(this->declare_parameter<double>("ndt_step_size", 0.1)),
  ndt_transformation_epsilon_(this->declare_parameter<double>("ndt_transformation_epsilon", 0.01)),
  ndt_max_iterations_(this->declare_parameter<int>("ndt_max_iterations", 35)),
  voxel_leaf_size_(this->declare_parameter<double>("voxel_leaf_size", 0.5)),
  transform_tolerance_sec_(this->declare_parameter<double>("transform_tolerance_sec", 0.3)),
  map_xy_margin_m_(this->declare_parameter<double>("map_xy_margin_m", 0.0)),
  publish_tf_(this->declare_parameter<bool>("publish_tf", true)),
  wait_initialpose_(this->declare_parameter<bool>("wait_initialpose", true)),
  initialpose_2d_(this->declare_parameter<std::string>("initialpose_2d", "0 0 0"))
{
  const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
  map_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    map_cloud_topic_, map_qos,
    std::bind(&pbl_global_localization::map_cloud_callback, this, std::placeholders::_1));

  source_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    source_cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&pbl_global_localization::source_cloud_callback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    initial_pose_topic_, rclcpp::QoS(10).reliable(),
    std::bind(&pbl_global_localization::initial_pose_callback, this, std::placeholders::_1));

  alignment_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(alignment_period_sec_),
    std::bind(&pbl_global_localization::alignment_timer_callback, this));

  if (publish_tf_) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

  if (!wait_initialpose_) {
    const auto initial_pose = parse_initialpose_2d(initialpose_2d_, map_frame_id_);
    if (initial_pose.has_value()) {
      latest_initial_pose_ = *initial_pose;
      has_initial_pose_ = true;
      force_initial_alignment_ = true;
      RCLCPP_INFO(
        this->get_logger(), "Using initialpose_2d as the initial pose: '%s'",
        initialpose_2d_.c_str());
    } else {
      RCLCPP_WARN(
        this->get_logger(), "Failed to parse initialpose_2d='%s'. Falling back to origin.",
        initialpose_2d_.c_str());
      latest_initial_pose_ = geometry_msgs::msg::PoseWithCovarianceStamped();
      latest_initial_pose_.header.frame_id = map_frame_id_;
      latest_initial_pose_.pose.pose.orientation.w = 1.0;
      has_initial_pose_ = true;
      force_initial_alignment_ = true;
    }
  }

  RCLCPP_INFO(this->get_logger(), "pbl_global_localization initialized");
  RCLCPP_INFO(this->get_logger(), "map_cloud_topic: %s", map_cloud_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "source_cloud_topic: %s", source_cloud_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "initial_pose_topic: %s", initial_pose_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "map_frame_id: %s", map_frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "odom_frame_id: %s", odom_frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "base_frame_id: %s", base_frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "source_buffer_duration_sec: %.3f", source_buffer_duration_sec_);
  RCLCPP_INFO(this->get_logger(), "alignment_period_sec: %.3f", alignment_period_sec_);
  RCLCPP_INFO(this->get_logger(), "ndt_resolution: %.3f", ndt_resolution_);
  RCLCPP_INFO(this->get_logger(), "ndt_step_size: %.3f", ndt_step_size_);
  RCLCPP_INFO(this->get_logger(), "ndt_transformation_epsilon: %.3f", ndt_transformation_epsilon_);
  RCLCPP_INFO(this->get_logger(), "ndt_max_iterations: %d", ndt_max_iterations_);
  RCLCPP_INFO(this->get_logger(), "voxel_leaf_size: %.3f", voxel_leaf_size_);
  RCLCPP_INFO(this->get_logger(), "transform_tolerance_sec: %.3f", transform_tolerance_sec_);
  RCLCPP_INFO(this->get_logger(), "map_xy_margin_m: %.3f", map_xy_margin_m_);
  RCLCPP_INFO(this->get_logger(), "wait_initialpose: %s", wait_initialpose_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "initialpose_2d: %s", initialpose_2d_.c_str());
  RCLCPP_INFO(this->get_logger(), "publish_tf: %s", publish_tf_ ? "true" : "false");
}

void pbl_global_localization::map_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (msg->header.frame_id != map_frame_id_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "map cloud frame_id is '%s' but expected '%s'.", msg->header.frame_id.c_str(),
      map_frame_id_.c_str());
  }

  auto cloud = std::make_shared<pcl::PointCloud<PointT>>();
  pcl::fromROSMsg(*msg, *cloud);
  if (voxel_leaf_size_ > 0.0) {
    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(
      static_cast<float>(voxel_leaf_size_), static_cast<float>(voxel_leaf_size_),
      static_cast<float>(voxel_leaf_size_));
    voxel.setInputCloud(cloud);
    auto filtered = std::make_shared<pcl::PointCloud<PointT>>();
    voxel.filter(*filtered);
    cloud = filtered;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  *map_cloud_ = *cloud;
  has_map_cloud_ = !map_cloud_->empty();
  if (has_map_cloud_) {
    map_xy_min_ = Eigen::Vector2d(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    map_xy_max_ = Eigen::Vector2d(-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity());
    for (const auto & point : map_cloud_->points) {
      map_xy_min_.x() = std::min(map_xy_min_.x(), static_cast<double>(point.x));
      map_xy_min_.y() = std::min(map_xy_min_.y(), static_cast<double>(point.y));
      map_xy_max_.x() = std::max(map_xy_max_.x(), static_cast<double>(point.x));
      map_xy_max_.y() = std::max(map_xy_max_.y(), static_cast<double>(point.y));
    }
    has_map_xy_bounds_ = true;
  }
}

void pbl_global_localization::source_cloud_callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (msg->header.frame_id != odom_frame_id_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "source cloud frame_id is '%s' but expected '%s'.", msg->header.frame_id.c_str(),
      odom_frame_id_.c_str());
  }

  const bool has_stamp = msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0;
  const rclcpp::Time stamp = has_stamp ? rclcpp::Time(msg->header.stamp) : this->now();

  auto cloud = std::make_shared<pcl::PointCloud<PointT>>();
  pcl::fromROSMsg(*msg, *cloud);
  if (voxel_leaf_size_ > 0.0) {
    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(
      static_cast<float>(voxel_leaf_size_), static_cast<float>(voxel_leaf_size_),
      static_cast<float>(voxel_leaf_size_));
    voxel.setInputCloud(cloud);
    auto filtered = std::make_shared<pcl::PointCloud<PointT>>();
    voxel.filter(*filtered);
    cloud = filtered;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  source_cloud_buffer_.push_back(BufferedCloud{cloud, stamp});

  const rclcpp::Time cutoff = stamp - rclcpp::Duration::from_seconds(source_buffer_duration_sec_);
  while (!source_cloud_buffer_.empty() && source_cloud_buffer_.front().stamp < cutoff) {
    source_cloud_buffer_.pop_front();
  }
}

void pbl_global_localization::initial_pose_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (msg->header.frame_id != map_frame_id_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "initial pose frame_id is '%s' but expected '%s'.", msg->header.frame_id.c_str(),
      map_frame_id_.c_str());
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_initial_pose_ = *msg;
    has_initial_pose_ = true;
    force_initial_alignment_ = true;
  }

  if (!try_align("initial_pose")) {
    RCLCPP_INFO(this->get_logger(), "Initial pose received, waiting for enough data to align.");
  }
}

void pbl_global_localization::alignment_timer_callback()
{
  try_align("timer");
}

bool pbl_global_localization::try_align(const char * trigger_reason)
{
  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
  pcl::PointCloud<PointT>::Ptr map_cloud;
  std::deque<BufferedCloud> source_buffer;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_map_cloud_ || (!has_initial_pose_ && wait_initialpose_)) {
      return false;
    }
    if (source_cloud_buffer_.empty()) {
      return false;
    }
    initial_pose = latest_initial_pose_;
    map_cloud = std::make_shared<pcl::PointCloud<PointT>>(*map_cloud_);
    source_buffer = source_cloud_buffer_;
  }

  auto source_cloud = std::make_shared<pcl::PointCloud<PointT>>();
  for (const auto & entry : source_buffer) {
    *source_cloud += *entry.cloud;
  }

  if (source_cloud->empty() || map_cloud->empty()) {
    return false;
  }

  if (voxel_leaf_size_ > 0.0) {
    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(
      static_cast<float>(voxel_leaf_size_), static_cast<float>(voxel_leaf_size_),
      static_cast<float>(voxel_leaf_size_));
    voxel.setInputCloud(source_cloud);
    auto filtered = std::make_shared<pcl::PointCloud<PointT>>();
    voxel.filter(*filtered);
    source_cloud = filtered;
  }

  const rclcpp::Time reference_stamp = source_buffer.back().stamp;
  Eigen::Isometry3d initial_guess = Eigen::Isometry3d::Identity();
  bool use_initial_pose_guess = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    use_initial_pose_guess = !has_valid_alignment_ || force_initial_alignment_;
  }

  if (use_initial_pose_guess) {
    const Eigen::Isometry3d initial_map_base = pose_to_transform(initial_pose.pose.pose);
    Eigen::Isometry3d odom_base = Eigen::Isometry3d::Identity();
    try {
      const auto transform = tf_buffer_->lookupTransform(
        odom_frame_id_, base_frame_id_, reference_stamp, rclcpp::Duration::from_seconds(0.1));
      odom_base = tf2::transformToEigen(transform.transform);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to lookup %s -> %s at t=%.3f, retrying with latest transform: %s",
        odom_frame_id_.c_str(), base_frame_id_.c_str(), reference_stamp.seconds(), ex.what());
      try {
        const auto transform = tf_buffer_->lookupTransform(
          odom_frame_id_, base_frame_id_, tf2::TimePointZero);
        odom_base = tf2::transformToEigen(transform.transform);
      } catch (const tf2::TransformException & latest_ex) {
        RCLCPP_WARN(
          this->get_logger(), "Failed to lookup latest %s -> %s transform: %s",
          odom_frame_id_.c_str(), base_frame_id_.c_str(), latest_ex.what());
        return false;
      }
    }

    if (!odom_base.matrix().allFinite()) {
      RCLCPP_WARN(
        this->get_logger(), "Invalid odom->base transform at t=%.3f.",
        reference_stamp.seconds());
      return false;
    }

    if (!is_inside_map_xy(initial_map_base.translation())) {
      RCLCPP_WARN(
        this->get_logger(),
        "Initial pose candidate (x=%.3f, y=%.3f) is outside map xy bounds, skipping NDT.",
        initial_map_base.translation().x(), initial_map_base.translation().y());
      return false;
    }

    initial_guess = initial_map_base * odom_base.inverse();
  } else {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_inside_map_xy(last_map_to_odom_.translation())) {
      RCLCPP_WARN(
        this->get_logger(),
        "Last map->odom estimate (x=%.3f, y=%.3f) is outside map xy bounds, skipping NDT.",
        last_map_to_odom_.translation().x(), last_map_to_odom_.translation().y());
      return false;
    }
    initial_guess = last_map_to_odom_;
  }

  pcl::NormalDistributionsTransform<PointT, PointT> ndt;
  ndt.setTransformationEpsilon(ndt_transformation_epsilon_);
  ndt.setStepSize(ndt_step_size_);
  ndt.setResolution(static_cast<float>(ndt_resolution_));
  ndt.setMaximumIterations(ndt_max_iterations_);
  ndt.setInputTarget(map_cloud);
  ndt.setInputSource(source_cloud);

  pcl::PointCloud<PointT> aligned_cloud;
  ndt.align(aligned_cloud, initial_guess.matrix().cast<float>());

  if (!ndt.hasConverged()) {
    RCLCPP_WARN(
      this->get_logger(), "NDT did not converge on %s trigger. fitness score: %.6f",
      trigger_reason, ndt.getFitnessScore());
    return false;
  }

  Eigen::Isometry3d map_to_odom = Eigen::Isometry3d::Identity();
  map_to_odom.matrix() = ndt.getFinalTransformation().cast<double>();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_map_to_odom_ = map_to_odom;
    has_valid_alignment_ = true;
    force_initial_alignment_ = false;
  }

  publish_map_to_odom(map_to_odom, reference_stamp);

  RCLCPP_INFO(
    this->get_logger(), "Aligned map->odom on %s trigger. fitness score: %.6f", trigger_reason,
    ndt.getFitnessScore());
  return true;
}

void pbl_global_localization::publish_map_to_odom(
  const Eigen::Isometry3d & map_to_odom, const rclcpp::Time & stamp)
{
  if (!publish_tf_ || !tf_broadcaster_) {
    return;
  }

  geometry_msgs::msg::TransformStamped tf_msg = tf2::eigenToTransform(map_to_odom);
  tf_msg.header.stamp = stamp + rclcpp::Duration::from_seconds(
                                     alignment_period_sec_ + transform_tolerance_sec_);
  tf_msg.header.frame_id = map_frame_id_;
  tf_msg.child_frame_id = odom_frame_id_;
  tf_broadcaster_->sendTransform(tf_msg);
}

Eigen::Isometry3d pbl_global_localization::pose_to_transform(
  const geometry_msgs::msg::Pose & pose)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  tf2::fromMsg(pose, transform);
  return transform;
}

std::optional<geometry_msgs::msg::PoseWithCovarianceStamped>
pbl_global_localization::parse_initialpose_2d(
  const std::string & initialpose_2d, const std::string & frame_id)
{
  std::istringstream stream(initialpose_2d);
  double x = 0.0;
  double y = 0.0;
  double yaw_deg = 0.0;
  if (!(stream >> x >> y >> yaw_deg)) {
    return std::nullopt;
  }

  double extra = 0.0;
  if (stream >> extra) {
    return std::nullopt;
  }

  constexpr double kPi = 3.14159265358979323846;
  const double yaw_rad = yaw_deg * kPi / 180.0;

  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = frame_id;
  msg.pose.pose.position.x = x;
  msg.pose.pose.position.y = y;
  msg.pose.pose.position.z = 0.0;

  const Eigen::AngleAxisd yaw_rotation(yaw_rad, Eigen::Vector3d::UnitZ());
  const Eigen::Quaterniond q(yaw_rotation);
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();
  return msg;
}

bool pbl_global_localization::is_inside_map_xy(const Eigen::Vector3d & map_position) const
{
  if (!has_map_xy_bounds_) {
    return true;
  }

  const double min_x = map_xy_min_.x() - map_xy_margin_m_;
  const double min_y = map_xy_min_.y() - map_xy_margin_m_;
  const double max_x = map_xy_max_.x() + map_xy_margin_m_;
  const double max_y = map_xy_max_.y() + map_xy_margin_m_;

  return map_position.x() >= min_x && map_position.x() <= max_x &&
         map_position.y() >= min_y && map_position.y() <= max_y;
}

}  // namespace pbl_localization

RCLCPP_COMPONENTS_REGISTER_NODE(pbl_localization::pbl_global_localization)
