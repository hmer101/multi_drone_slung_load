#!/bin/bash
# Script to launch Gazebo with SDF world containing multiple drones and load. 
# Also launches corresponding number of SITL PX4 firmware instances and required connections.
# Assumes that firmware has already been built for (Ignition) Gazebo using: make px4_sitl gz_x500
# Assumes that firmware source code is in location FIRMWARE_DIR
# Assumes that world file is in gz folder level to tools folder that this script is placed in

# Author: Harvey Merton
# Date: 03/03/23


# SETUP
# Directories
FIRMWARE_DIR="~/repos/PX4-Autopilot"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
GZ_DIR="$(dirname "$SCRIPT_DIR")/gz"

# Options
HEADLESS=1

# Constants
PI=3.141592654

# Parameters
PX4_SYS_AUTOSTART=4001
PX4_GZ_MODEL=x500
PX4_GZ_MODEL_NAME=swarm/model/x500
PX4_GZ_WORLD=default_tethered

NUM_DRONES=3
START_DRONE_NUM=1
DUMMY_DRONE_NUM=$(($NUM_DRONES+$START_DRONE_NUM))


# FUNCTIONS
# Kills processes
function cleanup() {
	pkill -x px4
	pkill gzclient
	pkill gzserver
}

# Run gazebo and launch the world 
function gz_launch_world() {
    #cd $GZ_DIR
    # gnome-terminal --tab -- bash -c "gz sim ./world_multi_with_load.sdf" PX4_GZ_MODEL_POSE="-100,-100"
	gnome-terminal --tab -- bash -c "HEADLESS=$HEADLESS PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART PX4_GZ_MODEL=$PX4_GZ_MODEL PX4_GZ_MODEL_POSE="-50,-50" $FIRMWARE_DIR/build/px4_sitl_default/bin/px4 -i $DUMMY_DRONE_NUM" #PX4_GZ_WORLD=$PX4_GZ_WORLD
}

# Create PX4 SITL instances and connect to models in the world
function create_sitl_instances() {
	# Spawn all drones numbering from first selected number
	for (( i=$(($START_DRONE_NUM)); i<$(($NUM_DRONES + $START_DRONE_NUM)); i++ )); do
        MODEL_NAME="${PX4_GZ_MODEL_NAME}_${i}" 
		gnome-terminal --tab -- bash -c "HEADLESS=$HEADLESS PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART PX4_GZ_MODEL_NAME=$MODEL_NAME $FIRMWARE_DIR/build/px4_sitl_default/bin/px4 -i $i"
		sleep 1
	done
}

# RUN COMMANDS
# Spawn drones
#cleanup
gz_launch_world

sleep 10
create_sitl_instances
sleep 2

# Create multiple MAVSDK servers
cd $SCRIPT_DIR
#gnome-terminal --tab -- bash -c "./multi_mavsdk_server.sh -n $(($NUM_DRONES+1))"

# Create ROS2 agent
gnome-terminal --tab -- bash -c "cd ~/repos/PX4-Autopilot; MicroXRCEAgent udp4 -p 8888" #micro-ros-agent udp4 --port 8888"