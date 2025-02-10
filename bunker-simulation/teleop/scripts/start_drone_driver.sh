#!/bin/bash

# Check for argument
if [ -z "$1" ]; then
    echo "Usage: $0 <drone_id>"
    exit 1
fi

drone_id="$1"

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/$drone_id/cmd_vel