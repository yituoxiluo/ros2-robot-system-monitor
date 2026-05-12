# ROS2 Robot System Monitor

## 项目简介

本项目是一个基于 ROS2 Jazzy 的机器人状态采集、通信管理与异常诊断系统，面向机器人系统工程中的设备接入、状态监控、通信链路检查、异常诊断、日志记录和数据可视化场景。

项目模拟真实机器人系统中的底层数据链路，包括关节状态、IMU、电池电压、电机温度和设备通信状态等信息，并通过 ROS2 Topic 完成数据发布、状态订阅、通信健康统计、异常检测和 CSV 日志记录。

当前开发环境：Ubuntu 24.04 + ROS 2 Jazzy。

## 项目目标

- 搭建 ROS2 机器人状态采集软件框架；
- 模拟机器人底层设备数据，包括关节状态、IMU、电池和电机温度；
- 实现通信超时、数据频率异常、低电压、过温和关节角超限检测；
- 支持 CSV 数据记录，并可配合 rosbag2 录制 Topic；
- 支持 rqt_plot / PlotJuggler / Rviz 可视化；
- 为后续机械臂、移动机器人、无人机系统联调和具身智能数据采集打基础。

## 系统架构

```text
Simulated Device Layer
        |
        v
device_driver_node
        |
        v
communication_manager_node
        |
        v
diagnostic_monitor_node
        |
        v
logger_node
        |
        v
rqt_plot / PlotJuggler / Rviz / CSV / rosbag2
```

## 节点说明

| 节点 | 功能 |
| --- | --- |
| `device_driver_node` | 模拟底层设备驱动，发布电池、电机温度、关节状态、IMU 和设备在线状态 |
| `communication_manager_node` | 统计核心 Topic 的频率、超时和最近更新时间，发布通信健康诊断 |
| `diagnostic_monitor_node` | 检测低电压、过温、关节角超限、设备离线和通信异常，发布系统诊断 |
| `logger_node` | 将核心状态量和系统状态写入 CSV 日志 |

## 核心 Topic

| Topic | 类型 | 说明 |
| --- | --- | --- |
| `/robot/battery_voltage` | `std_msgs/msg/Float32` | 电池电压 |
| `/robot/motor_temperature` | `std_msgs/msg/Float32` | 电机温度 |
| `/robot/joint_states` | `sensor_msgs/msg/JointState` | 三关节位置、速度、力矩 |
| `/robot/imu` | `sensor_msgs/msg/Imu` | IMU 角速度和线加速度 |
| `/robot/device_status` | `std_msgs/msg/String` | 设备在线状态 |
| `/robot/topic_health` | `diagnostic_msgs/msg/DiagnosticArray` | 通信健康状态 |
| `/robot/system_diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 系统诊断状态 |
| `/robot/system_state` | `std_msgs/msg/String` | `OK` / `WARN` / `ERROR` |

## 快速运行

```bash
cd ~/robot_system_ws
colcon build --packages-select robot_system_monitor --symlink-install
source install/setup.bash
ros2 launch robot_system_monitor monitor.launch.py
```

查看系统状态：

```bash
ros2 topic echo /robot/system_state
ros2 topic echo /robot/system_diagnostics
```

查看 CSV 日志：

```bash
tail -f /tmp/robot_system_monitor/robot_state_log.csv
```

## 故障注入演示

项目提供了 `config/fault_demo.yaml`，会在运行数秒后模拟低电压、过温、关节角超限、IMU 通信中断和设备离线：

```bash
ros2 launch robot_system_monitor monitor.launch.py \
  config_file:=$(ros2 pkg prefix robot_system_monitor)/share/robot_system_monitor/config/fault_demo.yaml
```

此时 `/robot/system_state` 会从 `OK` 变为 `ERROR`，CSV 日志会写入 `/tmp/robot_system_monitor/fault_demo_log.csv`。

## 可视化与数据记录

```bash
rqt_plot /robot/battery_voltage/data /robot/motor_temperature/data
ros2 bag record /robot/battery_voltage /robot/motor_temperature /robot/joint_states /robot/imu /robot/system_state
```

## 项目亮点

- 使用 ROS2 多节点解耦模拟机器人底层数据链路；
- 基于 `diagnostic_msgs` 建立通信健康与系统诊断输出；
- 支持参数化故障注入，便于快速演示异常检测效果；
- CSV 日志可用于后续数据分析、曲线可视化或故障复盘；
- 项目结构贴近机器人系统工程中的设备接入、状态监控和联调流程。
