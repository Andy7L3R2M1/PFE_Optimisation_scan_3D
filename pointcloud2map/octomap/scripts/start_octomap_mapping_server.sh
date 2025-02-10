#!/bin/bash

# Check for argument
if [ -z "$1" ]; then
    echo "Usage: $0 <drone_name>"
    exit 1
fi

drone_name="$1"

# Octomap server produisant une matrice de voxels colorés (fichier de configuration: color_octomap_server_<drone_name>_launchfile)
ros2 launch octomap_server color_octomap_mapping_"$drone_name".launch.xml