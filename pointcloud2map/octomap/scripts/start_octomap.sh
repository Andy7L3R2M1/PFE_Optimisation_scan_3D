#!/bin/bash

# Function to check if a ROS 2 topic exists
function topic_exists() {
    ros2 topic list | grep -q "$1"
}

# Topic to check for
topic="/${DRONE_ID}/depth/points"

# Loop until the topic is available
while ! topic_exists "$topic"; do
    echo "Waiting for $topic to become available..."
    sleep 1
done

echo "$topic detected. Starting octomap..."

/root/ros_ws/scripts/start_octomap_mapping_server.sh ${DRONE_ID}