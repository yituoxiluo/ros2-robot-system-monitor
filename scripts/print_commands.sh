#!/usr/bin/env bash

cat <<'EOF'
ROS2 Robot System Monitor quick commands
========================================

1. Build the package
--------------------
source /opt/ros/jazzy/setup.bash
cd ~/robot_system_ws
colcon build --packages-select robot_system_monitor --symlink-install
source install/setup.bash

2. Launch normal monitor mode
-----------------------------
source /opt/ros/jazzy/setup.bash
cd ~/robot_system_ws
source install/setup.bash
ros2 launch robot_system_monitor monitor.launch.py

3. Launch fault demo mode
-------------------------
source /opt/ros/jazzy/setup.bash
cd ~/robot_system_ws
source install/setup.bash
ros2 launch robot_system_monitor monitor.launch.py \
  config_file:=$(ros2 pkg prefix robot_system_monitor)/share/robot_system_monitor/config/fault_demo.yaml

4. Check running nodes
----------------------
ros2 node list

Expected nodes:
/communication_manager_node
/device_driver_node
/diagnostic_monitor_node
/logger_node

5. Check robot topics
---------------------
ros2 topic list | grep /robot

6. Inspect raw simulated data
-----------------------------
ros2 topic echo /robot/battery_voltage --once
ros2 topic echo /robot/motor_temperature --once
ros2 topic echo /robot/device_status --once
ros2 topic echo /robot/joint_states --once
ros2 topic echo /robot/imu --once

7. Check topic frequency
------------------------
ros2 topic hz /robot/imu
ros2 topic hz /robot/joint_states

8. Check system diagnostics
---------------------------
ros2 topic echo /robot/system_state --once
ros2 topic echo /robot/system_diagnostics --once
ros2 topic echo /robot/topic_health --once

9. Watch CSV logs
-----------------
tail -f /tmp/robot_system_monitor/robot_state_log.csv
tail -f /tmp/robot_system_monitor/fault_demo_log.csv

10. Record rosbag2 data
-----------------------
ros2 bag record /robot/battery_voltage /robot/motor_temperature /robot/joint_states /robot/imu /robot/system_state

11. Plot data with rqt_plot
---------------------------
rqt_plot /robot/battery_voltage/data /robot/motor_temperature/data

EOF
