nav2:
	colcon build --packages-select nav2_test && ros2 launch nav2_test navigation_launch.py use_sim_time:=true
neu_lidar:
	colcon build --packages-select neu_lidar && ros2 launch neu_lidar neu_lidar_launch.py
