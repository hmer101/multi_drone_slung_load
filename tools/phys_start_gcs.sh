#!/bin/bash
# Script to spawn all terminals and programs used on the GCS when running a physical drone

# Author: Harvey Merton
# Date: 01/25/24

NUM_DRONES=1 #3  # Set the maximum number of drones to launch
START_DRONE_NUM=2 #1  # Set the starting drone number
drone_count=0  # Initialize a counter for launched drones
line_num=0  # Initialize a line counter

# Start QGC
gnome-terminal --tab -- bash -c "/home/harvey/.appImage/QGroundControl.AppImage" #; exec bash"

# Balena SSH into drones and view the logs
# Note must be on same network as drones (BilabRover_2.4GHz)
while IFS= read -r line
do
	# Start at START_DRONE_NUM and end at NUM_DRONES
	line_num=$((line_num + 1))

	if [ "$line_num" -lt "$START_DRONE_NUM" ]; then
		continue  # Skip lines until START_DRONE_NUM is reached
	fi

	if [ "$drone_count" -ge "$NUM_DRONES" ]; then
		echo "Maximum number of drones ($NUM_DRONES) reached. Exiting..."
		break
	fi

	# Extract info from txt file to ssh into drones
	drone_info=($line)
	drone_uuid=${drone_info[0]}
	drone_container_main=${drone_info[1]}
	
	# Note how the 'balena logs' command is 'piped' to the 'balena ssh' command
	gnome-terminal --tab -- bash -c "echo 'balena logs -f $drone_container_main; exit;' | balena ssh $drone_uuid.local" 

	# Increment the drone count
	drone_count=$((drone_count + 1))


done < ../config/phys_drones_info.txt #Note this file needs a blank line at the end. Contains 'device_uuid main_container_id' for each drone on a new line


# Run GCS ROS2 launch
gnome-terminal -- bash -c "ros2 launch swarm_load_carry phys_gcs.launch.py" #; exec bash"