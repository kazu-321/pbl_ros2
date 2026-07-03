# pbl_localization

This package provides localization-related nodes for the PBL robot.

## `pbl_wheel_odometry`

This node subscribes to `/joint_states` and publishes `/odometry`. TF output can be enabled or disabled with a parameter.

## Parameters
- `wheel_radius` (double, default: 0.05)
- `wheel_separation` (double, default: 0.46)
- `odom_frame_id` (string, default: "odom")
- `base_frame_id` (string, default: "base_link")
- `publish_tf` (bool, default: true)

## Topics
- Subscribes: `/joint_states` (`sensor_msgs/msg/JointState`)
- Publishes: `/odometry` (`nav_msgs/msg/Odometry`)
- Publishes TF: `odom` → `base_link` when `publish_tf` is `true`

## Usage
Add to your launch file or run as a component node.

## `pbl_pose_stamped_tf_broadcaster`

This node subscribes to TF and publishes `geometry_msgs/msg/PoseStamped` on `localization/current_pose` by default.

### Parameters
- `output_topic` (string, default: `/localization/current_pose`)
- `map_frame_id` (string, default: "map")
- `base_frame_id` (string, default: "base_link")
- `publish_rate_hz` (double, default: 10.0)
- `transform_timeout_sec` (double, default: 0.1)

### Topics
- Publishes: `output_topic` (`geometry_msgs/msg/PoseStamped`)

### Usage
Use this when another node already publishes TF for the robot pose and you want a normalized `PoseStamped` stream.
