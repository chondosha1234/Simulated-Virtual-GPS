#!/bin/bash

# define log directory and make if necessary 
LOG_DIR="$HOME/cs1980/logs"
mkdir -p "$LOG_DIR"

# function to clean up all processes on exit 
cleanup() {
	echo "Stopping all drone processes..."
	kill $PIDS 2>/dev/null
	exit 0
}

# trap SIGINT to cleanup before exit 
trap cleanup SIGINT


echo "Building and sourcing virtualGPS package..."

cd ~/cs1980/cs1980_ws && colcon build
source install/setup.bash 

echo "Launching raspimouse and gz simulation..."

ros2 launch virtual_gps raspimouse_default.launch.py > "$LOG_DIR/raspimouse.log" 2>&1 &
PIDS="$PIDS $!"

echo "Launching MAVROS connections..."

# mavros instances -- one per drone 
ros2 launch mavros px4.launch fcu_url:="udp://:14540@localhost:14557" namespace:="drone1" > "$LOG_DIR/mavros_drone1.log" 2>&1 & 
PIDS="$PIDS $!"
ros2 launch mavros px4.launch fcu_url:="udp://:14541@localhost:14558" namespace:="drone2" > "$LOG_DIR/mavros_drone2.log" 2>&1 & 
PIDS="$PIDS $!"
ros2 launch mavros px4.launch fcu_url:="udp://:14542@localhost:14559" namespace:="drone3" > "$LOG_DIR/mavros_drone3.log" 2>&1 & 
PIDS="$PIDS $!"
ros2 launch mavros px4.launch fcu_url:="udp://:14543@localhost:14560" namespace:="drone4" > "$LOG_DIR/mavros_drone4.log" 2>&1 & 
PIDS="$PIDS $!"

echo "Launching QGroundControl..."

# start QGroundControl 
cd ~/Downloads && ./QGroundControl.AppImage > "$LOG_DIR/qgc.log" 2>&1 & 
PIDS="$PIDS $!"

sleep 5

echo "Launching PX4 models for the X500 drones..."

#cd ~/PX4-Autopilot && make px4_sitl gz_x500 > "$LOG_DIR/px4_drone1.log" 2>&1 &
#PIDS="$PIDS $!"
cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,3" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1 > "$LOG_DIR/px4_drone1.log" 2>&1 &
PIDS="$PIDS $!"
cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="3,0" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 2 > "$LOG_DIR/px4_drone2.log" 2>&1 & 
PIDS="$PIDS $!"
cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,-3" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 3 > "$LOG_DIR/px4_drone3.log" 2>&1 & 
PIDS="$PIDS $!"
cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="-3,0" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 4 > "$LOG_DIR/px4_drone4.log" 2>&1 & 
PIDS="$PIDS $!"

sleep 2

echo "Bridging gz topics to ROS topics"

ros2 run ros_gz_bridge parameter_bridge /model/x500_1/pose@geometry_msgs/msg/Pose@gz.msgs.Pose &
PIDS="$PIDS $!"
ros2 run ros_gz_bridge parameter_bridge /model/x500_2/pose@geometry_msgs/msg/Pose@gz.msgs.Pose &
PIDS="$PIDS $!"
ros2 run ros_gz_bridge parameter_bridge /model/x500_3/pose@geometry_msgs/msg/Pose@gz.msgs.Pose &
PIDS="$PIDS $!"
ros2 run ros_gz_bridge parameter_bridge /model/x500_4/pose@geometry_msgs/msg/Pose@gz.msgs.Pose &
PIDS="$PIDS $!"

ros2 run ros_gz_bridge parameter_bridge /model/raspimouse/pose@geometry_msgs/msg/Pose@gz.msgs.Pose &
PIDS="$PIDS $!"

echo "Launching ROS2 launch file 'virtual_gps.launch.py'"

source ~/cs1980/cs1980_ws/install/setup.bash
ros2 launch virtual_gps virtual_gps.launch.py > "$LOG_DIR/ros2_launch.log" 2>&1 &
PIDS="$PIDS $!"

echo "All drones and systems started."
echo "Press Ctrl+C to shut everything down."

# keep script running until user stops 
wait

