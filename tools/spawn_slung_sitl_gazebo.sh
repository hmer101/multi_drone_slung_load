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

# Constants
PI=3.141592654

# Parameters
PX4_SYS_AUTOSTART=4001
PX4_GZ_MODEL=connected_group/model/x500 #swarm/model/x500
PX4_SIM_MODEL=x500

NUM_DRONES=3
START_DRONE_NUM=1


# FUNCTIONS
# Kills processes
function cleanup() {
	pkill -x px4
	pkill gzclient
	pkill gzserver
}

# Run gazebo and launch the world 
function gz_launch_world() {
    cd $GZ_DIR
    gnome-terminal --tab -- bash -c "ign gazebo -r ./world_multi_with_load.sdf"
}

# Create PX4 SITL instances and connect to models in the world
function create_sitl_instances() {
	# Spawn all drones numbering from first selected number
	for (( i=$START_DRONE_NUM; i<$(($NUM_DRONES + $START_DRONE_NUM)); i++ )); do
        MODEL_NAME="${PX4_GZ_MODEL}_${i}"
		gnome-terminal --tab -- bash -c "PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART PX4_GZ_MODEL_NAME=$MODEL_NAME PX4_SIM_MODEL=$PX4_SIM_MODEL $FIRMWARE_DIR/build/px4_sitl_default/bin/px4 -i $i"
	done
}

# RUN COMMANDS
# Spawn drones
cleanup
gz_launch_world

sleep 5
create_sitl_instances
sleep 2

# Create multiple MAVSDK servers
cd $SCRIPT_DIR
gnome-terminal --tab -- bash -c "./multi_mavsdk_server.sh -n $NUM_DRONES"

# Create ROS2 agent
gnome-terminal --tab -- bash -c "cd ~/repos/PX4-Autopilot; micro-ros-agent udp4 --port 8888"