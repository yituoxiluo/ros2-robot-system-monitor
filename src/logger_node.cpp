#include <algorithm>
#include <chrono>
#include <filesystem>
#include <functional>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

class LoggerNode : public rclcpp::Node
{
public:
  LoggerNode()
  : Node("logger_node")
  {
    log_path_ = this->declare_parameter<std::string>(
      "log_path", "/tmp/robot_system_monitor/robot_state_log.csv");
    log_period_sec_ = this->declare_parameter<double>("log_period_sec", 0.2);

    open_log_file();

    const auto qos = rclcpp::SensorDataQoS();
    battery_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/robot/battery_voltage", qos,
      [this](std_msgs::msg::Float32::SharedPtr msg) {
        battery_voltage_ = msg->data;
        has_battery_ = true;
      });
    motor_temp_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/robot/motor_temperature", qos,
      [this](std_msgs::msg::Float32::SharedPtr msg) {
        motor_temperature_ = msg->data;
        has_motor_temperature_ = true;
      });
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/robot/joint_states", qos,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        joint_positions_ = msg->position;
        has_joint_state_ = true;
      });
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/robot/imu", qos,
      [this](sensor_msgs::msg::Imu::SharedPtr msg) {
        imu_linear_acceleration_ = {
          msg->linear_acceleration.x,
          msg->linear_acceleration.y,
          msg->linear_acceleration.z};
        imu_angular_velocity_z_ = msg->angular_velocity.z;
        has_imu_ = true;
      });
    system_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot/system_state", 10,
      [this](std_msgs::msg::String::SharedPtr msg) {
        system_state_ = msg->data;
      });

    const auto log_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(std::max(log_period_sec_, 0.05)));
    timer_ = this->create_wall_timer(log_period, std::bind(&LoggerNode::write_row, this));

    RCLCPP_INFO(this->get_logger(), "logger_node started, writing %s.", log_path_.c_str());
  }

private:
  void open_log_file()
  {
    const std::filesystem::path path(log_path_);
    if (path.has_parent_path()) {
      std::filesystem::create_directories(path.parent_path());
    }

    log_file_.open(log_path_, std::ios::out | std::ios::trunc);
    if (!log_file_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "failed to open log file: %s", log_path_.c_str());
      return;
    }

    log_file_
      << "timestamp_sec,battery_voltage,motor_temperature,"
      << "joint1_position,joint2_position,joint3_position,"
      << "imu_ax,imu_ay,imu_az,imu_wz,system_state\n";
  }

  void write_optional(bool has_value, double value)
  {
    if (has_value) {
      log_file_ << std::fixed << std::setprecision(4) << value;
    }
  }

  void write_joint_position(std::size_t index)
  {
    if (has_joint_state_ && joint_positions_.size() > index) {
      log_file_ << std::fixed << std::setprecision(4) << joint_positions_[index];
    }
  }

  void write_row()
  {
    if (!log_file_.is_open()) {
      return;
    }

    log_file_ << std::fixed << std::setprecision(4) << this->now().seconds() << ",";
    write_optional(has_battery_, battery_voltage_);
    log_file_ << ",";
    write_optional(has_motor_temperature_, motor_temperature_);
    log_file_ << ",";
    write_joint_position(0);
    log_file_ << ",";
    write_joint_position(1);
    log_file_ << ",";
    write_joint_position(2);
    log_file_ << ",";
    write_optional(has_imu_, imu_linear_acceleration_[0]);
    log_file_ << ",";
    write_optional(has_imu_, imu_linear_acceleration_[1]);
    log_file_ << ",";
    write_optional(has_imu_, imu_linear_acceleration_[2]);
    log_file_ << ",";
    write_optional(has_imu_, imu_angular_velocity_z_);
    log_file_ << "," << system_state_ << "\n";
    log_file_.flush();
  }

  std::string log_path_;
  double log_period_sec_;
  std::ofstream log_file_;

  double battery_voltage_ = 0.0;
  double motor_temperature_ = 0.0;
  std::vector<double> joint_positions_;
  std::vector<double> imu_linear_acceleration_ = {0.0, 0.0, 0.0};
  double imu_angular_velocity_z_ = 0.0;
  std::string system_state_ = "UNKNOWN";

  bool has_battery_ = false;
  bool has_motor_temperature_ = false;
  bool has_joint_state_ = false;
  bool has_imu_ = false;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr motor_temp_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoggerNode>());
  rclcpp::shutdown();
  return 0;
}
