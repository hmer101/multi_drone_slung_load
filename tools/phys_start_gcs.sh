#!/bin/bash
# Script to spawn all terminals and programs used on the GCS when running a physical drone

# Author: Harvey Merton
# Date: 01/25/24

NUM_DRONES=3 # Set the maximum number of drones to launch
START_DRONE_NUM=1  # Set the starting drone number
NUM_LOAD=1  # Set the number of loads to be used

# Function to SSH into devices and view logs
ssh_and_view_logs() {
    local device_type=$1
    local num_devices=$2
    local start_num=$3
    local uuid_file=$4

    local count=0  # Initialize a counter for launched devices
    local line_num=0  # Initialize a line counter

    while IFS= read -r line; do
        # Start at start_num and end at num_devices
        line_num=$((line_num + 1))

        if [ "$line_num" -lt "$start_num" ]; then
            continue  # Skip lines until start_num is reached
        fi

        if [ "$count" -ge "$num_devices" ]; then
            echo "Maximum number of $device_type ($num_devices) reached. Exiting..."
            break
        fi

        # Extract info from txt file to ssh into devices
        device_uuid=${line}

        # Get container info from balena
        device_container_main=$(echo 'balena container ls --format "{{.ID}}" | head -n 1' | balena ssh $device_uuid.local)

        # Open gnome-terminal tab to view logs
        gnome-terminal --tab -- bash -c "echo 'balena logs $device_container_main -f; exit;' | balena ssh $device_uuid.local"

        # Increment the device count
        count=$((count + 1))
    done < "$uuid_file"
}

# Start QGC
bash -c "/home/harvey/.appImage/QGroundControl.AppImage" #gnome-terminal --tab -- 

# Balena SSH into drones and view the logs
ssh_and_view_logs "drones" $NUM_DRONES $START_DRONE_NUM "../config/phys_drones_uuid.txt"

# Balena SSH into loads and view the logs
ssh_and_view_logs "loads" $NUM_LOAD 1 "../config/phys_load_uuid.txt"