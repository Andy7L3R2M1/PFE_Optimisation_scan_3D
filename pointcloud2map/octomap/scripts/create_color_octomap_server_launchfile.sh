#!/bin/bash

# Check for argument
if [ -z "$1" ]; then
    echo "Usage: $0 <drone_name>"
    exit 1
fi

drone_name="$1"

echo "Creating color octomap server launchfile for drone: $drone_name"

# Create launchfile
cat <<EOF > /root/ros_ws/install/octomap_server/share/octomap_server/launch/color_octomap_mapping_"$drone_name".launch.xml
<?xml version="1.0"?>
<launch>
    <node pkg="octomap_server" exec="color_octomap_server_node" name="color_octomap_server_$drone_name">
        <param name="resolution" value="0.05" />

        <!-- fixed map frame (set to 'map' if SLAM or localization running!) -->
        <param name="frame_id" value="$drone_name/odom" />

        <!-- maximum range to integrate (speedup!) -->
        <param name="sensor_model.max_range" value="8.0" />

        <param name="height_map" value="false" />
        <param name="colored_map" value="true" />

        <!-- data source to integrate (PointCloud2) -->
        <remap from="cloud_in" to="$drone_name/depth/points" />

        <!-- data output (MarkerArray) -->
        <remap from="occupied_cells_vis_array" to="$drone_name/occupied_cells_vis_array" />

    </node>
</launch>
EOF