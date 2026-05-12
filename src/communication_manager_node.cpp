#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

class CommunicationManagerNode : public rclcpp::Node
{
public:
  CommunicationManagerNode()
  : Node("communication_manager_node")
  {
    timeout_sec_ = this->declare_parameter<double>("timeout_sec", 0.6);
    min_frequency_hz_ = this->declare_parameter<double>("min_frequency_hz", 5.0);
    report_period_sec_ = this->declare_parameter<double>("report_period_sec", 1.0);

    register_topic("/robot/battery_voltage", min_frequency_hz_);
    register_topic("/robot/motor_temperature", min_frequency_hz_);
    register_topic("/robot/joint_states", min_frequency_hz_);
    register_topic("/robot/imu", min_frequency_hz_);
    register_topic("/robot/device_status", min_frequency_hz_);

    const auto qos = rclcpp::SensorDataQoS();
    battery_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/robot/battery_voltage", qos,
      [this](std_msgs::msg::Float32::SharedPtr) { mark_topic("/robot/battery_voltage"); });
    motor_temp_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/robot/motor_temperature", qos,
      [this](std_msgs::msg::Float32::SharedPtr) { mark_topic("/robot/motor_temperature"); });
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/robot/joint_states", qos,
      [this](sensor_msgs::msg::JointState::SharedPtr) { mark_topic("/robot/joint_states"); });
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/robot/imu", qos,
      [this](sensor_msgs::msg::Imu::SharedPtr) { mark_topic("/robot/imu"); });
    device_status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot/device_status", qos,
      [this](std_msgs::msg::String::SharedPtr) { mark_topic("/robot/device_status"); });

    diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/robot/topic_health", 10);

    last_report_time_ = this->now();
    const auto report_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(std::max(report_period_sec_, 0.1)));
    timer_ = this->create_wall_timer(
      report_period, std::bind(&CommunicationManagerNode::publish_diagnostics, this));

    RCLCPP_INFO(this->get_logger(), "communication_manager_node started.");
  }

private:
  struct TopicHealth
  {
    double expected_min_hz;
    rclcpp::Time last_msg_time;
    std::size_t messages_in_window = 0;
    bool seen = false;
  };

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

  void register_topic(const std::string & topic_name, double expected_min_hz)
  {
    TopicHealth health;
    health.expected_min_hz = expected_min_hz;
    topics_.emplace(topic_name, health);
  }

  void mark_topic(const std::string & topic_name)
  {
    auto iter = topics_.find(topic_name);
    if (iter == topics_.end()) {
      return;
    }

    iter->second.last_msg_time = this->now();
    iter->second.messages_in_window++;
    iter->second.seen = true;
  }

  void publish_diagnostics()
  {
    const rclcpp::Time now = this->now();
    const double elapsed_sec = std::max((now - last_report_time_).seconds(), 1e-6);

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now;

    for (auto & [topic_name, health] : topics_) {
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "communication" + topic_name;
      status.hardware_id = "simulated_robot";

      const double frequency_hz =
        static_cast<double>(health.messages_in_window) / elapsed_sec;
      const double age_sec = health.seen ?
        (now - health.last_msg_time).seconds() :
        std::numeric_limits<double>::infinity();

      if (!health.seen || age_sec > timeout_sec_) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = health.seen ? "topic timeout" : "no message received";
      } else if (frequency_hz < health.expected_min_hz) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "low publish frequency";
      } else {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "topic healthy";
      }

      status.values.push_back(make_key_value("topic", topic_name));
      status.values.push_back(make_key_value("frequency_hz", format_double(frequency_hz)));
      status.values.push_back(make_key_value("expected_min_hz", format_double(health.expected_min_hz)));
      status.values.push_back(make_key_value(
        "age_sec", std::isfinite(age_sec) ? format_double(age_sec) : "inf"));
      status.values.push_back(make_key_value("timeout_sec", format_double(timeout_sec_)));

      array.status.push_back(status);
      health.messages_in_window = 0;
    }

    diagnostics_pub_->publish(array);
    last_report_time_ = now;
  }

  std::map<std::string, TopicHealth> topics_;
  rclcpp::Time last_report_time_;

  double timeout_sec_;
  double min_frequency_hz_;
  double report_period_sec_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr motor_temp_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr device_status_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommunicationManagerNode>());
  rclcpp::shutdown();
  return 0;
}
