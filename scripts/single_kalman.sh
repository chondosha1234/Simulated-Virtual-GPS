#!/bin/bash

# define log directory and make if necessary 
LOG_DIR="$HOME/Simulated-Virtual-GPS/logs"
mkdir -p "$LOG_DIR"

# function to clean up all processes on exit 
cleanup() {
	echo "Stopping all drone processes..."
	kill $PIDS 2>/dev/null
	exit 0
}

# trap SIGINT to cleanup before exit 
trap cleanup SIGINT


echo "Building ROS 2 package and sourcing..."

cd ~/Simulated-Virtual-GPS/cs1980_ws && colcon build
source install/setup.bash 


echo "Launching raspimouse and gz sim..."

ros2 launch virtual_gps raspimouse_default.launch.py > "$LOG_DIR/raspimouse.log" 2>&1 &
PIDS="$PIDS $!"

sleep 3


echo "Launching MAVROS connections..."

# mavros instances -- one per drone -- tgt_system is needed so the mavros namespace actually connects to other drone
ros2 launch mavros px4.launch fcu_url:="udp://:14540@localhost:14557" namespace:="/x500_0" > "$LOG_DIR/mavros_drone0.log" 2>&1 & 
PIDS="$PIDS $!"


echo "Launching QGroundControl..."

# start QGroundControl 
cd ~/Downloads && ./QGroundControl.AppImage > "$LOG_DIR/qgc.log" 2>&1 & 
PIDS="$PIDS $!"


echo "Launching PX4 models for the X500 drones..."

cd ~/PX4-Autopilot && PX4_GZ_MODEL_POSE="3,0" make px4_sitl gz_x500 > "$LOG_DIR/px4_drone0.log" 2>&1 &
PIDS="$PIDS $!"

sleep 2


echo "Bridging gz topics to ROS topics..."

ros2 run ros_gz_bridge parameter_bridge /model/x500_0/pose@geometry_msgs/msg/TransformStamped@gz.msgs.Pose &
PIDS="$PIDS $!"

ros2 run ros_gz_bridge parameter_bridge /model/raspimouse/pose@geometry_msgs/msg/TransformStamped@gz.msgs.Pose &
PIDS="$PIDS $!"


echo "Launching ROS2 launch file 'virtual_gps_v2.launch.py'"

sleep 2

source ~/Simulated-Virtual-GPS/cs1980_ws/install/setup.bash
ros2 launch virtual_gps kalman_gps_single.launch.py &
PIDS="$PIDS $!"

sleep 14

ros2 launch virtual_gps kalman.launch.py > "$LOG_DIR/kalman_launch.log" 2>&1 &
PIDS="$PIDS $!"

# wait some time for all topics and params to be established before trying to set speed
echo "Setting drone speed parameters..."

ros2 param set /x500_0/param MPC_XY_VEL_MAX 2.0


echo "All drones and systems started."
echo "Press Ctrl+C to shut everything down."

# keep script running until user stops 
wait
