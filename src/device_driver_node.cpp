#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

class DeviceDriverNode : public rclcpp::Node
{
public:
  DeviceDriverNode()
  : Node("device_driver_node"), time_count_(0.0)
  {
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 10.0);
    battery_nominal_voltage_ = this->declare_parameter<double>("battery_nominal_voltage", 24.0);
    battery_drop_rate_v_per_s_ = this->declare_parameter<double>("battery_drop_rate_v_per_s", 0.01);
    motor_initial_temperature_ = this->declare_parameter<double>("motor_initial_temperature", 35.0);
    motor_rise_rate_c_per_s_ = this->declare_parameter<double>("motor_rise_rate_c_per_s", 0.08);
    joint_motion_amplitude_ = this->declare_parameter<double>("joint_motion_amplitude", 1.0);

    inject_faults_ = this->declare_parameter<bool>("inject_faults", false);
    fault_start_sec_ = this->declare_parameter<double>("fault_start_sec", 15.0);
    low_fault_voltage_ = this->declare_parameter<double>("low_fault_voltage", 20.0);
    high_fault_temperature_ = this->declare_parameter<double>("high_fault_temperature", 78.0);
    joint_fault_amplitude_ = this->declare_parameter<double>("joint_fault_amplitude", 2.0);
    drop_imu_after_sec_ = this->declare_parameter<double>("drop_imu_after_sec", -1.0);
    device_offline_after_sec_ = this->declare_parameter<double>("device_offline_after_sec", -1.0);

    battery_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/robot/battery_voltage", 10);
    motor_temp_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/robot/motor_temperature", 10);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/robot/joint_states", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      "/robot/imu", 10);
    device_status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/robot/device_status", 10);

    const double safe_rate_hz = std::clamp(publish_rate_hz_, 1.0, 200.0);
    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / safe_rate_hz));

    timer_ = this->create_wall_timer(
      timer_period, std::bind(&DeviceDriverNode::publish_robot_state, this));

    RCLCPP_INFO(
      this->get_logger(), "device_driver_node started at %.1f Hz.", safe_rate_hz);
  }

private:
  void publish_robot_state()
  {
    time_count_ += 1.0 / std::clamp(publish_rate_hz_, 1.0, 200.0);

    publish_battery_voltage();
    publish_motor_temperature();
    publish_joint_states();
    publish_imu_data();
    publish_device_status();
  }

  void publish_battery_voltage()
  {
    auto msg = std_msgs::msg::Float32();
    double voltage = battery_nominal_voltage_ - battery_drop_rate_v_per_s_ * time_count_;

    if (inject_faults_ && time_count_ >= fault_start_sec_) {
      voltage = std::min(voltage, low_fault_voltage_);
    }

    msg.data = static_cast<float>(voltage);
    battery_pub_->publish(msg);
  }

  void publish_motor_temperature()
  {
    auto msg = std_msgs::msg::Float32();
    double temperature = motor_initial_temperature_ + motor_rise_rate_c_per_s_ * time_count_;

    if (inject_faults_ && time_count_ >= fault_start_sec_) {
      temperature = std::max(temperature, high_fault_temperature_);
    }

    msg.data = static_cast<float>(temperature);
    motor_temp_pub_->publish(msg);
  }

  void publish_joint_states()
  {
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    msg.name = {"joint1", "joint2", "joint3"};

    double amplitude = joint_motion_amplitude_;
    if (inject_faults_ && time_count_ >= fault_start_sec_) {
      amplitude = joint_fault_amplitude_;
    }

    msg.position = {
      amplitude * std::sin(time_count_),
      amplitude * std::cos(time_count_),
      0.5 * amplitude * std::sin(time_count_)
    };
    msg.velocity = {
      amplitude * std::cos(time_count_),
      -amplitude * std::sin(time_count_),
      0.5 * amplitude * std::cos(time_count_)
    };
    msg.effort = {0.0, 0.0, 0.0};

    joint_state_pub_->publish(msg);
  }

  void publish_imu_data()
  {
    if (drop_imu_after_sec_ > 0.0 && time_count_ >= drop_imu_after_sec_) {
      return;
    }

    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = this->now();
    msg.header.frame_id = "imu_link";

    msg.angular_velocity.x = 0.01 * std::sin(time_count_);
    msg.angular_velocity.y = 0.02 * std::cos(time_count_);
    msg.angular_velocity.z = 0.03;
    msg.linear_acceleration.x = 0.1 * std::sin(time_count_);
    msg.linear_acceleration.y = 0.1 * std::cos(time_count_);
    msg.linear_acceleration.z = 9.8;

    imu_pub_->publish(msg);
  }

  void publish_device_status()
  {
    auto msg = std_msgs::msg::String();
    msg.data = "ONLINE";

    if (device_offline_after_sec_ > 0.0 && time_count_ >= device_offline_after_sec_) {
      msg.data = "OFFLINE";
    }

    device_status_pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motor_temp_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr device_status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double time_count_;
  double publish_rate_hz_;
  double battery_nominal_voltage_;
  double battery_drop_rate_v_per_s_;
  double motor_initial_temperature_;
  double motor_rise_rate_c_per_s_;
  double joint_motion_amplitude_;
  bool inject_faults_;
  double fault_start_sec_;
  double low_fault_voltage_;
  double high_fault_temperature_;
  double joint_fault_amplitude_;
  double drop_imu_after_sec_;
  double device_offline_after_sec_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeviceDriverNode>());
  rclcpp::shutdown();
  return 0;
}
