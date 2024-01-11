#!/bin/bash
# Script to spawn multiple drones in (Ignition) Gazebo
# Assumes that firmware has already been built for (Ignition) Gazebo using: make px4_sitl gz_x500
# Assumes that firmware source code is in location FIRMWARE_DIR

# Author: Harvey Merton
# Date: 01/04/23


# SETUP
# Directories
FIRMWARE_DIR="~/repos/PX4-Autopilot"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Constants
PI=3.141592654

# Parameters
PX4_SYS_AUTOSTART=4001
PX4_GZ_MODEL=x500

NUM_DRONES=1 #3
START_DRONE_NUM=1
SPAWN_CENTER_X=0
SPAWN_CENTER_Y=0
SPAWN_R=1.5

SPAWN_PTS_X=()
SPAWN_PTS_Y=()
SPAWN_Z=0.25


# FUNCTIONS
# Kills processes
function cleanup() {
	pkill -x px4
	pkill gzclient
	pkill gzserver
}

# Generates X and Y co-ordinates to spawn NUM_DRONES equally spaced in a circle around the SPAWN_CENTER 
function generate_spawn_points() {
	
	# Use polar co-ordinate system to generate (x,y) points in circumscribing circle 
	for (( i=0;i<$NUM_DRONES;i++ )); do
		SPAWN_PTS_X+=($(echo "$SPAWN_R*c(2*$PI*$i/$NUM_DRONES)+$SPAWN_CENTER_X"| bc -l)) 
		SPAWN_PTS_Y+=($(echo "$SPAWN_R*s(2*$PI*$i/$NUM_DRONES)+$SPAWN_CENTER_Y"| bc -l))
	done	
}

# Spawns drones using defined spawn points
function spawn_drones() {
	# Spawn all drones numbering from first selected number
	for (( i=$START_DRONE_NUM; i<$(($NUM_DRONES + $START_DRONE_NUM)); i++ )); do
		gnome-terminal --tab -- bash -c "PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART PX4_GZ_MODEL_POSE="${SPAWN_PTS_X[$(($i-$START_DRONE_NUM))]},${SPAWN_PTS_Y[$(($i-$START_DRONE_NUM))]},$SPAWN_Z" PX4_GZ_MODEL=$PX4_GZ_MODEL $FIRMWARE_DIR/build/px4_sitl_default/bin/px4 -i $i"

		# Wait longer for first drone as Gazebo takes time to start
		if [ $i == $START_DRONE_NUM ]; then
			sleep 10 #If Gazebo already started, laptop works with 2, conservative at 10
		else
			sleep 2 #If Gazebo already started, laptop works with 0.5
		fi

	done
}


# RUN COMMANDS
# Spawn drones
cleanup
generate_spawn_points
spawn_drones

sleep 2

# Create multiple MAVSDK servers
cd $SCRIPT_DIR
#gnome-terminal --tab -- bash -c "./multi_mavsdk_server.sh -n $NUM_DRONES"

# Create ROS2 agent (note must install first: sudo snap install micro-xrce-dds-agent --edge)
gnome-terminal --tab -- bash -c "micro-xrce-dds-agent udp4 -p 8888" #"cd ~/repos/PX4-Autopilot; micro-ros-agent udp4 --port 8888"
