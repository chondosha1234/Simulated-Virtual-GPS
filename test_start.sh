#!/bin/bash

# define log directory and make if necessary 
LOG_DIR="$HOME/logs"
mkdir -p "$LOG_DIR"

# function to clean up all processes on exit 
cleanup() {
	echo "Stopping all drone processes..."
	kill $PIDS 2>/dev/null
	exit 0
}

# trap SIGINT to cleanup before exit 
trap cleanup SIGINT

echo "Launching MAVROS connections..."

# mavros instances -- one per drone 
ros2 launch mavros px4.launch fcu_url:="udp://:14540@localhost:14557" namespace:="drone1" > "$LOG_DIR/mavros_drone1.log" 2>&1 & 
PIDS="$PIDS $!"
ros2 launch mavros px4.launch fcu_url:="udp://:14541@localhost:14558" namespace:="drone2" > "$LOG_DIR/mavros_drone2.log" 2>&1 & 
PIDS="$PIDS $!"


echo "Launching PX4 models for the X500 drones..."


cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,3" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1 > "$LOG_DIR/px4_drone1.log" 2>&1 &
PIDS="$PIDS $!"
# sleep and wait for gazebo to open, so subsequent drones are added to the same instance 
sleep 5

cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,0" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 2 > "$LOG_DIR/px4_drone2.log" 2>&1 & 
PIDS="$PIDS $!"
sleep 2

echo "Bridging gz topics to ROS topics"

ros2 run ros_gz_bridge parameter_bridge /model/x500_0/pose@geometry_msgs/msg/Pose@gz.msgs.Pose &
PIDS="$PIDS $!"
ros2 run ros_gz_bridge parameter_bridge /model/x500_1/pose@geometry_msgs/msg/Pose@gz.msgs.Pose &
PIDS="$PIDS $!"

echo "All drones and systems started."
echo "Press Ctrl+C to shut everything down."

# keep script running until user stops 
wait

