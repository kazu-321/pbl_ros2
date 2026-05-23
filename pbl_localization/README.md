# pbl_localization

Odometry calculation node for the PBL robot. This node subscribes to `/joint_states` and publishes `/odometry`. TF output can be enabled or disabled with a parameter.

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
