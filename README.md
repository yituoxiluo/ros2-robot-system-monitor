
# ROS2 Robot System Monitor

## 项目简介

本项目是一个基于 ROS2 的机器人状态采集、通信管理与可视化系统，面向机器人系统工程中的设备接入、状态监控、异常诊断、日志记录和数据可视化场景。

项目模拟真实机器人系统中的底层数据链路，包括关节状态、IMU、电池电压、电机温度和设备通信状态等信息，并通过 ROS2 Topic 完成数据发布、状态订阅、异常检测和日志记录。

本项目目前处于设计与开发阶段，后续将在 Ubuntu + ROS2 Humble 环境下进行功能验证。

## 项目目标

- 搭建 ROS2 机器人状态采集软件框架；
- 模拟机器人底层设备数据，包括关节状态、IMU、电池和电机温度；
- 实现通信超时、数据频率异常、低电压、过温和关节角超限检测；
- 支持 CSV / rosbag2 数据记录；
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
rqt_plot / PlotJuggler / Rviz / CSV
