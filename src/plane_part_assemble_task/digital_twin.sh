#!/bin/bash

# 启动 ROS launch 文件
gnome-terminal -x bash -c "roslaunch plane_part_assemble_task run_task.launch"

gnome-terminal -x bash -c "roslaunch plane_part_stewart_driver stewart_bringup.launch"

# 启动 plane_part_with_agv.py 脚本
gnome-terminal -x bash -c "python3 ./scripts/plane_part_with_agv.py; exec bash"

# 启动 agv_remote_controller.py 脚本
gnome-terminal -x bash -c "python3 ./scripts/agv_remote_controller.py; exec bash"
