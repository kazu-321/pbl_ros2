#ifndef PBL_COMMUNICATION__USB_COMMUNICATION_NODE_HPP_
#define PBL_COMMUNICATION__USB_COMMUNICATION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <cstdint>
#include <string>

namespace pbl_communication {

class usb_communication_node : public rclcpp::Node {
   public:
    explicit usb_communication_node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~usb_communication_node() override;

   private:
    struct WheelCommand {
        int32_t right_rpm = 0;
        int32_t left_rpm = 0;
        bool valid = false;
    };

    void joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void poll_serial();
    void reconnect_serial();
    bool open_serial();
    void close_serial();
    bool write_command_line(int32_t right_rpm, int32_t left_rpm);
    void publish_joint_state(int32_t right_rpm, int32_t left_rpm);
    bool parse_feedback_line(const std::string & line, int32_t & right_rpm, int32_t & left_rpm) const;
    static std::string trim_line(std::string line);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr poll_timer_;
    rclcpp::TimerBase::SharedPtr reconnect_timer_;

    std::string port_;
    int64_t baud_rate_;
    int serial_fd_;
    bool serial_open_;
    rclcpp::Time last_feedback_stamp_;
    rclcpp::Time last_reconnect_attempt_;

    WheelCommand pending_command_;
    bool has_pending_command_;
    int32_t last_right_rpm_;
    int32_t last_left_rpm_;
};

}  // namespace pbl_communication

#endif  // PBL_COMMUNICATION__USB_COMMUNICATION_NODE_HPP_
