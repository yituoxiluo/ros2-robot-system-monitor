#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

class DiagnosticMonitorNode : public rclcpp::Node
{
public:
  DiagnosticMonitorNode()
  : Node("diagnostic_monitor_node")
  {
    battery_warn_voltage_ = this->declare_parameter<double>("battery_warn_voltage", 22.0);
    battery_error_voltage_ = this->declare_parameter<double>("battery_error_voltage", 20.5);
    motor_warn_temperature_ = this->declare_parameter<double>("motor_warn_temperature", 65.0);
    motor_error_temperature_ = this->declare_parameter<double>("motor_error_temperature", 80.0);
    joint_position_limit_rad_ = this->declare_parameter<double>("joint_position_limit_rad", 1.57);
    stale_data_timeout_sec_ = this->declare_parameter<double>("stale_data_timeout_sec", 1.5);
    diagnostic_period_sec_ = this->declare_parameter<double>("diagnostic_period_sec", 1.0);

    const auto qos = rclcpp::SensorDataQoS();
    battery_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/robot/battery_voltage", qos,
      [this](std_msgs::msg::Float32::SharedPtr msg) {
        battery_voltage_ = msg->data;
        battery_stamp_ = this->now();
        has_battery_ = true;
      });
    motor_temp_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/robot/motor_temperature", qos,
      [this](std_msgs::msg::Float32::SharedPtr msg) {
        motor_temperature_ = msg->data;
        motor_temp_stamp_ = this->now();
        has_motor_temperature_ = true;
      });
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/robot/joint_states", qos,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        joint_positions_ = msg->position;
        joint_stamp_ = this->now();
        has_joint_state_ = true;
      });
    device_status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot/device_status", qos,
      [this](std_msgs::msg::String::SharedPtr msg) {
        device_status_ = msg->data;
        device_status_stamp_ = this->now();
        has_device_status_ = true;
      });
    topic_health_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/robot/topic_health", 10,
      [this](diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
        latest_topic_health_ = *msg;
        topic_health_stamp_ = this->now();
        has_topic_health_ = true;
      });

    diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/robot/system_diagnostics", 10);
    system_state_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/robot/system_state", 10);

    const auto diagnostic_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(std::max(diagnostic_period_sec_, 0.1)));
    timer_ = this->create_wall_timer(
      diagnostic_period, std::bind(&DiagnosticMonitorNode::publish_diagnostics, this));

    RCLCPP_INFO(this->get_logger(), "diagnostic_monitor_node started.");
  }

private:
  static diagnostic_msgs::msg::KeyValue make_key_value(
    const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    return kv;
  }

  static std::string format_double(double value, int precision = 2)
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
  }

  static const char * level_to_state(uint8_t level)
  {
    if (level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
      return "ERROR";
    }
    if (level >= diagnostic_msgs::msg::DiagnosticStatus::WARN) {
      return "WARN";
    }
    return "OK";
  }

  static uint8_t merge_level(uint8_t lhs, uint8_t rhs)
  {
    return std::max(lhs, rhs);
  }

  diagnostic_msgs::msg::DiagnosticStatus make_status(
    const std::string & name, uint8_t level, const std::string & message)
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = name;
    status.hardware_id = "simulated_robot";
    status.level = level;
    status.message = message;
    return status;
  }

  bool is_stale(const rclcpp::Time & stamp, const rclcpp::Time & now) const
  {
    return (now - stamp).seconds() > stale_data_timeout_sec_;
  }

  diagnostic_msgs::msg::DiagnosticStatus check_battery(const rclcpp::Time & now)
  {
    if (!has_battery_ || is_stale(battery_stamp_, now)) {
      return make_status(
        "system/battery", diagnostic_msgs::msg::DiagnosticStatus::ERROR, "battery data stale");
    }

    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string message = "battery normal";
    if (battery_voltage_ <= battery_error_voltage_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      message = "battery critically low";
    } else if (battery_voltage_ <= battery_warn_voltage_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      message = "battery low";
    }

    auto status = make_status("system/battery", level, message);
    status.values.push_back(make_key_value("voltage", format_double(battery_voltage_)));
    status.values.push_back(make_key_value("warn_voltage", format_double(battery_warn_voltage_)));
    status.values.push_back(make_key_value("error_voltage", format_double(battery_error_voltage_)));
    return status;
  }

  diagnostic_msgs::msg::DiagnosticStatus check_motor_temperature(const rclcpp::Time & now)
  {
    if (!has_motor_temperature_ || is_stale(motor_temp_stamp_, now)) {
      return make_status(
        "system/motor_temperature", diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "temperature data stale");
    }

    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string message = "motor temperature normal";
    if (motor_temperature_ >= motor_error_temperature_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      message = "motor temperature critical";
    } else if (motor_temperature_ >= motor_warn_temperature_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      message = "motor temperature high";
    }

    auto status = make_status("system/motor_temperature", level, message);
    status.values.push_back(make_key_value("temperature_c", format_double(motor_temperature_)));
    status.values.push_back(make_key_value("warn_temperature_c", format_double(motor_warn_temperature_)));
    status.values.push_back(make_key_value("error_temperature_c", format_double(motor_error_temperature_)));
    return status;
  }

  diagnostic_msgs::msg::DiagnosticStatus check_joint_limits(const rclcpp::Time & now)
  {
    if (!has_joint_state_ || is_stale(joint_stamp_, now) || joint_positions_.empty()) {
      return make_status(
        "system/joint_limits", diagnostic_msgs::msg::DiagnosticStatus::ERROR, "joint data stale");
    }

    double max_abs_position = 0.0;
    for (const auto position : joint_positions_) {
      max_abs_position = std::max(max_abs_position, std::abs(position));
    }

    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string message = "joint position normal";
    if (max_abs_position > joint_position_limit_rad_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      message = "joint position out of limit";
    }

    auto status = make_status("system/joint_limits", level, message);
    status.values.push_back(make_key_value("max_abs_position_rad", format_double(max_abs_position)));
    status.values.push_back(make_key_value("limit_rad", format_double(joint_position_limit_rad_)));
    return status;
  }

  diagnostic_msgs::msg::DiagnosticStatus check_device_status(const rclcpp::Time & now)
  {
    if (!has_device_status_ || is_stale(device_status_stamp_, now)) {
      return make_status(
        "system/device_status", diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "device status stale");
    }

    const bool online = device_status_ == "ONLINE";
    auto status = make_status(
      "system/device_status",
      online ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      online ? "device online" : "device offline");
    status.values.push_back(make_key_value("status", device_status_));
    return status;
  }

  diagnostic_msgs::msg::DiagnosticStatus check_communication(const rclcpp::Time & now)
  {
    if (!has_topic_health_ || is_stale(topic_health_stamp_, now)) {
      return make_status(
        "system/communication", diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "topic health not ready");
    }

    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    int ok_count = 0;
    int warn_count = 0;
    int error_count = 0;

    for (const auto & topic_status : latest_topic_health_.status) {
      level = merge_level(level, topic_status.level);
      if (topic_status.level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
        error_count++;
      } else if (topic_status.level >= diagnostic_msgs::msg::DiagnosticStatus::WARN) {
        warn_count++;
      } else {
        ok_count++;
      }
    }

    std::string message = "communication normal";
    if (level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
      message = "communication error";
    } else if (level >= diagnostic_msgs::msg::DiagnosticStatus::WARN) {
      message = "communication warning";
    }

    auto status = make_status("system/communication", level, message);
    status.values.push_back(make_key_value("ok_topics", std::to_string(ok_count)));
    status.values.push_back(make_key_value("warn_topics", std::to_string(warn_count)));
    status.values.push_back(make_key_value("error_topics", std::to_string(error_count)));
    return status;
  }

  void publish_diagnostics()
  {
    const rclcpp::Time now = this->now();
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now;

    uint8_t system_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    for (auto status : {
        check_battery(now),
        check_motor_temperature(now),
        check_joint_limits(now),
        check_device_status(now),
        check_communication(now)}) {
      system_level = merge_level(system_level, status.level);
      array.status.push_back(status);
    }

    std_msgs::msg::String state_msg;
    state_msg.data = level_to_state(system_level);
    system_state_pub_->publish(state_msg);
    diagnostics_pub_->publish(array);

    if (system_level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "robot system state: %s", state_msg.data.c_str());
    } else if (system_level >= diagnostic_msgs::msg::DiagnosticStatus::WARN) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "robot system state: %s", state_msg.data.c_str());
    }
  }

  double battery_warn_voltage_;
  double battery_error_voltage_;
  double motor_warn_temperature_;
  double motor_error_temperature_;
  double joint_position_limit_rad_;
  double stale_data_timeout_sec_;
  double diagnostic_period_sec_;

  double battery_voltage_ = 0.0;
  double motor_temperature_ = 0.0;
  std::vector<double> joint_positions_;
  std::string device_status_ = "UNKNOWN";
  diagnostic_msgs::msg::DiagnosticArray latest_topic_health_;

  bool has_battery_ = false;
  bool has_motor_temperature_ = false;
  bool has_joint_state_ = false;
  bool has_device_status_ = false;
  bool has_topic_health_ = false;

  rclcpp::Time battery_stamp_;
  rclcpp::Time motor_temp_stamp_;
  rclcpp::Time joint_stamp_;
  rclcpp::Time device_status_stamp_;
  rclcpp::Time topic_health_stamp_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr motor_temp_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr device_status_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr topic_health_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiagnosticMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
