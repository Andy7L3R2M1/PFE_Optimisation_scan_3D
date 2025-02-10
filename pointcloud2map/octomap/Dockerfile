FROM gitlab.polytech.umontpellier.fr:5050/eii/ros/ros-vanilla:main

COPY src /root/ros_ws/src
COPY scripts /root/ros_ws/scripts

RUN apt update \
    && apt install -y ros-humble-octomap ros-humble-pcl-ros

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-select octomap_msgs octomap_ros octomap_server"

CMD ["/bin/bash", "-c", "scripts/create_color_octomap_server_launchfile.sh ${DRONE_ID} && source /opt/ros/humble/setup.bash && source install/setup.bash && scripts/start_octomap.sh"]