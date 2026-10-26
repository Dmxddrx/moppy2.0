#!/bin/bash

echo "=========================================="
echo "    Starting Moppy LiDAR & SLAM Stack     "
echo "=========================================="

# 1. Set Serial Port Permissions
echo "[1/4] Setting USB permissions..."
sudo chmod 666 /dev/ttyUSB0

# Source ROS 2 environment
source /opt/ros/lyrical/setup.bash

# 2. Launch TF Static Publishers in Background
echo "[2/4] Starting TF Frame Tree..."
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id odom --child-frame-id base_footprint &
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id base_footprint --child-frame-id base_link &
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id laser_frame &

sleep 2

# 3. Launch LiDAR Driver Node in Background
echo "[3/4] Launching LiDAR Driver Node..."
python3 ~/moppy_lidar_node.py &

sleep 3

# 4. Launch SLAM Toolbox in Background
echo "[4/4] Launching SLAM Toolbox..."
ros2 launch slam_toolbox online_async_launch.py &

sleep 3

# 5. Launch RViz2
echo "Opening RViz2..."
rviz2

# Clean up all background processes when RViz2 is closed
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT
