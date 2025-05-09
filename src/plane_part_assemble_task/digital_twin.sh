gnome-terminal -x bash -c "python3 ./scripts/plane_part_with_agv.py"
gnome-terminal -x bash -c "python3 ./scripts/agv_controller.py"
roslaunch plane_part_assemble_task run_task.launch