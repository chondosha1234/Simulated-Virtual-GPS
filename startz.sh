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
ros2 launch mavros px4.launch fcu_url:="udp://:14542@localhost:14559" namespace:="drone3" > "$LOG_DIR/mavros_drone3.log" 2>&1 & 
PIDS="$PIDS $!"
ros2 launch mavros px4.launch fcu_url:="udp://:14543@localhost:14560" namespace:="drone4" > "$LOG_DIR/mavros_drone4.log" 2>&1 & 
PIDS="$PIDS $!"

echo "Launching QGroundControl..."

# start QGroundControl 
# cd ~/Downloads && ./QGroundControl.AppImage > "$LOG_DIR/qgc.log" 2>&1 & 
flatpak run org.mavlink.qgroundcontrol > "$LOG_DIR/qgc.log" 2>&1 & 
PIDS="$PIDS $!"

echo "Launching PX4 models for the X500 drones..."

#cd ~/PX4-Autopilot && make px4_sitl gz_x500 > "$LOG_DIR/px4_drone1.log" 2>&1 &
#PIDS="$PIDS $!"
cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,3" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1 > "$LOG_DIR/px4_drone1.log" 2>&1 &
#gnome-terminal -- bash -c "cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,3" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1; exec bash" 
PIDS="$PIDS $!"
# sleep and wait for gazebo to open, so subsequent drones are added to the same instance 
sleep 8

cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="3,0" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 2 > "$LOG_DIR/px4_drone2.log" 2>&1 & 
PIDS="$PIDS $!"
cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,-3" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 3 > "$LOG_DIR/px4_drone3.log" 2>&1 & 
PIDS="$PIDS $!"
cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="-3,0" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 4 > "$LOG_DIR/px4_drone4.log" 2>&1 & 
PIDS="$PIDS $!"

echo "All drones and systems started."
echo "Press Ctrl+C to shut everything down."

# keep script running until user stops 
wait

