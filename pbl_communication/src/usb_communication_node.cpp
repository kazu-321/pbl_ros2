#include "pbl_communication/usb_communication_node.hpp"

#include "rclcpp_components/register_node_macro.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <array>
#include <fcntl.h>
#include <sstream>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace pbl_communication {
namespace {
constexpr const char *kRightWheelJointName = "wheel_right";
constexpr const char *kLeftWheelJointName = "wheel_left";
constexpr const char *kFeedbackTopic = "/joint_states";
constexpr const char *kCommandTopic = "/joint_commands";
constexpr std::array<const char *, 2> kCandidatePorts = {"/dev/ttyACM0", "/dev/ttyACM1"};
constexpr int kReadBufferSize = 256;
constexpr double kPi = 3.14159265358979323846;
}  // namespace

usb_communication_node::usb_communication_node(const rclcpp::NodeOptions & options)
    : Node("usb_communication_node", options),
      port_(this->declare_parameter<std::string>("port", "")),
      baud_rate_(this->declare_parameter<int64_t>("baud_rate", 115200)),
      serial_fd_(-1),
      serial_open_(false),
      last_feedback_stamp_(this->now()),
      last_reconnect_attempt_(this->now()),
      has_pending_command_(false),
      last_right_rpm_(0),
      last_left_rpm_(0) {
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(kFeedbackTopic, rclcpp::SensorDataQoS());
    joint_command_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        kCommandTopic, rclcpp::QoS(10),
        std::bind(&usb_communication_node::joint_command_callback, this, std::placeholders::_1));

    reconnect_serial();
    poll_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&usb_communication_node::poll_serial, this));
    reconnect_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&usb_communication_node::reconnect_serial, this));

    RCLCPP_INFO(this->get_logger(), "usb_communication_node initialized");
    RCLCPP_INFO(this->get_logger(), "port: %s", port_.empty() ? "auto(ttyACM0, ttyACM1)" : port_.c_str());
    RCLCPP_INFO(this->get_logger(), "baud_rate: %ld", static_cast<long>(baud_rate_));
}

usb_communication_node::~usb_communication_node() {
    close_serial();
}

void usb_communication_node::joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "joint_commands received: name_count=%zu velocity_count=%zu",
    //             msg->name.size(), msg->velocity.size());
    double right = 0.0;
    double left = 0.0;
    bool found_right = false;
    bool found_left = false;
    for (std::size_t i = 0; i < msg->name.size(); ++i) {
        if (i < msg->velocity.size()) {
            if (msg->name[i] == kRightWheelJointName) {
                right = msg->velocity[i];
                found_right = true;
            }
            if (msg->name[i] == kLeftWheelJointName) {
                left = msg->velocity[i];
                found_left = true;
            }
        }
    }
    if (!found_right || !found_left) {
        if (msg->velocity.size() >= 2) {
            right = msg->velocity[0];
            left = msg->velocity[1];
        } else {
            return;
        }
    }

    pending_command_.right_rpm = static_cast<int32_t>(std::lround(right * 60.0 / (2.0 * kPi)));
    pending_command_.left_rpm = static_cast<int32_t>(std::lround(left * 60.0 / (2.0 * kPi)));
    pending_command_.valid = true;
    has_pending_command_ = true;
    // RCLCPP_INFO(this->get_logger(), "Queued USB command: left=%ld rpm right=%ld rpm",
    //             static_cast<long>(pending_command_.left_rpm), static_cast<long>(pending_command_.right_rpm));
}

bool usb_communication_node::open_serial() {
    close_serial();
    if (port_.empty()) {
        return false;
    }
    serial_fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
        return false;
    }

    termios tty {};
    if (tcgetattr(serial_fd_, &tty) != 0) {
        close_serial();
        return false;
    }

    cfmakeraw(&tty);
    tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    tty.c_cflag |= static_cast<tcflag_t>(CS8);
    tty.c_iflag &= static_cast<tcflag_t>(~(IXON | IXOFF | IXANY));
    tty.c_oflag &= static_cast<tcflag_t>(~OPOST);
    tty.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO | ECHOE | ISIG));
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    speed_t speed = B115200;
    if (baud_rate_ != 115200) {
        RCLCPP_WARN(this->get_logger(), "Only 115200 is configured in code; requested %ld.", static_cast<long>(baud_rate_));
    }
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        close_serial();
        return false;
    }

    serial_open_ = true;
    return true;
}

void usb_communication_node::close_serial() {
    if (serial_fd_ >= 0) {
        ::close(serial_fd_);
        serial_fd_ = -1;
    }
    serial_open_ = false;
}

void usb_communication_node::reconnect_serial() {
    if (serial_open_) {
        return;
    }

    const rclcpp::Time now = this->now();
    if ((now - last_reconnect_attempt_).seconds() < 1.0) {
        return;
    }
    last_reconnect_attempt_ = now;

    if (!port_.empty()) {
        if (open_serial()) {
            RCLCPP_INFO(this->get_logger(), "Reconnected to %s", port_.c_str());
        }
        return;
    }

    for (const char *candidate : kCandidatePorts) {
        port_ = candidate;
        if (open_serial()) {
            RCLCPP_INFO(this->get_logger(), "Connected to %s", port_.c_str());
            return;
        }
    }

    port_.clear();
}

bool usb_communication_node::write_command_line(int32_t right_rpm, int32_t left_rpm) {
    if (!serial_open_ && !open_serial()) {
        return false;
    }
    char buffer[64];
    const int len = std::snprintf(buffer, sizeof(buffer), "%+d,%+d\n", left_rpm, right_rpm);
    if (len <= 0) {
        return false;
    }
    const ssize_t written = ::write(serial_fd_, buffer, static_cast<std::size_t>(len));
    if (written == len) {
        // RCLCPP_INFO(this->get_logger(), "USB TX: %s", buffer);
        return true;
    }
    RCLCPP_WARN(this->get_logger(), "USB TX failed on %s", port_.c_str());
    return false;
}

bool usb_communication_node::parse_feedback_line(const std::string & line, int32_t & right_rpm, int32_t & left_rpm) const {
    const auto comma = line.find(',');
    if (comma == std::string::npos) {
        return false;
    }
    try {
        right_rpm = std::stoi(line.substr(0, comma));
        left_rpm = std::stoi(line.substr(comma + 1));
        return true;
    } catch (...) {
        return false;
    }
}

std::string usb_communication_node::trim_line(std::string line) {
    line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
    line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
    return line;
}

void usb_communication_node::publish_joint_state(int32_t right_rpm, int32_t left_rpm) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    msg.name = {kRightWheelJointName, kLeftWheelJointName};
    const double right_rad_s = static_cast<double>(right_rpm) * 2.0 * kPi / 60.0;
    const double left_rad_s = static_cast<double>(left_rpm) * 2.0 * kPi / 60.0;
    msg.velocity = {right_rad_s, left_rad_s};
    msg.position = {0.0, 0.0};
    msg.effort = {0.0, 0.0};
    joint_state_pub_->publish(msg);
}

void usb_communication_node::poll_serial() {
    if (!serial_open_) {
        return;
    }

    if (has_pending_command_) {
        // RCLCPP_INFO(this->get_logger(), "Sending pending USB command: left=%ld rpm right=%ld rpm",
        //             static_cast<long>(pending_command_.left_rpm), static_cast<long>(pending_command_.right_rpm));
        if (write_command_line(pending_command_.right_rpm, pending_command_.left_rpm)) {
            has_pending_command_ = false;
        }
    }

    char buffer[kReadBufferSize];
    while (true) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(serial_fd_, &readfds);
        timeval timeout {0, 0};
        const int ret = select(serial_fd_ + 1, &readfds, nullptr, nullptr, &timeout);
        if (ret <= 0 || !FD_ISSET(serial_fd_, &readfds)) {
            break;
        }

        const ssize_t bytes = ::read(serial_fd_, buffer, sizeof(buffer) - 1);
        if (bytes <= 0) {
            if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                break;
            }
            RCLCPP_WARN(this->get_logger(), "USB serial disconnected from %s", port_.c_str());
            close_serial();
            return;
        }
        buffer[bytes] = '\0';
        std::stringstream ss(buffer);
        std::string line;
        while (std::getline(ss, line)) {
            line = trim_line(line);
            if (line.empty()) {
                continue;
            }
            int32_t right_rpm = 0;
            int32_t left_rpm = 0;
            if (parse_feedback_line(line, right_rpm, left_rpm)) {
                last_right_rpm_ = right_rpm;
                last_left_rpm_ = left_rpm;
                last_feedback_stamp_ = this->now();
                // RCLCPP_INFO(this->get_logger(), "USB RX: %+d,%+d", left_rpm, right_rpm);
                publish_joint_state(right_rpm, left_rpm);
            }
        }
    }
}

}  // namespace pbl_communication

RCLCPP_COMPONENTS_REGISTER_NODE(pbl_communication::usb_communication_node)
