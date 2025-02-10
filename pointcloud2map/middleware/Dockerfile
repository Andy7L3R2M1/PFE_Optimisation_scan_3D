FROM gitlab.polytech.umontpellier.fr:5050/eii/ros/ros-vanilla:main

COPY src /root/ros_ws/src

COPY requirements.txt requirements.txt
RUN pip install --upgrade pip && pip install -r requirements.txt

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-select pol_msgs pol_db_management pol_spatial_data_types pol_spatial_data_middleware --symlink-install"

#CMD ["/bin/bash", "-c", "source install/setup.bash && ros2 run pol_spatial_data_middleware middleware --ros-args -p drone_id:=${DRONE_ID}"]
CMD ["/bin/bash"]
