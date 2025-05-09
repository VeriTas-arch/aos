#!/bin/bash

# 启动 plane_part_with_agv.py 脚本
gnome-terminal -- bash -c "python3 ./scripts/plane_part_with_agv.py; exec bash"

# 启动 agv_controller.py 脚本
gnome-terminal -- bash -c "python3 ./scripts/agv_controller.py; exec bash"

# 启动 ROS launch 文件
roslaunch plane_part_assemble_task run_task.launch