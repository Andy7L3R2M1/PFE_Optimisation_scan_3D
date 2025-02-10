#!/usr/bin/env python

import rclpy
import os
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from pol_spatial_data_types.SpatialData_DB_Connector import SpatialData_DB_Connector
from pol_spatial_data_types.SpatialData import SpatialData
from pol_spatial_data_types.SpatialPoint import SpatialPoint
from pol_spatial_data_types.ColorRGBANormalized import ColorRGBANormalized
from std_srvs.srv import Empty

class Middleware(Node):

    def __init__(self):
        super().__init__("middleware")
        self.get_logger().info("Starting middleware initialization")
        self.declare_parameter("drone_id", "UNDEFINED")
        self.spatial_data: SpatialData = SpatialData()
        SpatialData_DB_Connector.instance()
        self.drone_id = (
            self.get_parameter("drone_id").get_parameter_value().string_value
        )
        if self.drone_id == "UNDEFINED":
            self.get_logger().error("Drone ID not defined")
            raise ValueError("Drone ID not defined")
        self.subscriber = self.create_subscription(
            MarkerArray,
            f"{self.drone_id}/occupied_cells_vis_array",
            self.marker_array_subscriber,
            10,
        )
        #le service qui reset octomap
        service='color_octomap_server_'+self.drone_id+'/reset'
        self.srv_reset_octo = self.create_client(Empty, service)
        while not self.srv_reset_octo.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request = Empty.Request()
        self.clear_index=0



        self.get_logger().info(f"Middleware initialized for {self.drone_id}")

    def marker_array_subscriber(self, msg: MarkerArray):
        # clear octomap_server buffer (choose one of the following commands depending on the octomap server used)
        # os.system("ros2 service call /octomap_server/reset std_srvs/Empty")
        # os.system("ros2 service call /color_octomap_server/reset std_srvs/Empty")
        
        # os.system(
        #     f"ros2 service call /color_octomap_server_{self.drone_id}/reset std_srvs/Empty"
        # )
        self.clear_index=self.clear_index+1
        if self.clear_index>10:
            self.clear_index=0
            self.future=self.srv_reset_octo.call_async(self.request)
            self.get_logger().info("send clear service")
        # self.future.add_done_callback(self.handle_response) # ? necessaire ??

        points = []

        # register each point in the database
        for marker in msg.markers:
            if len(marker.points)>0 :
                # print number of points in the marker
                self.get_logger().info(
                    f"Received new marker with {len(marker.points)} points"
                )
                for point in marker.points:
                    # self.get_logger().info(f"Received new point: {point}")
                    # get index of the point in the marker array
                    index = marker.points.index(point)
                    # if the point has a specific color, use it, otherwise use the marker color
                    try:
                        color = marker.colors[index]
                    except IndexError:
                        color = marker.color
                    spatial_point = SpatialPoint(
                        # convert coordinates from meters to centimeters
                        int(point.x * pow(10, 2)),
                        int(point.y * pow(10, 2)),
                        int(point.z * pow(10, 2)),
                        # convert color to normalized RGBA [0-255]
                        ColorRGBANormalized(color),
                    )
                    points.append(spatial_point)

                    """ Register one point in the database
                    # self.get_logger().info(f"Created new point: {spatial_point}")
                    self.spatial_data.register_point(spatial_point)
                    # store the point in the database
                    SpatialData_DB_Connector.instance().store_point(spatial_point)
                    """

        # store the points in the database
        SpatialData_DB_Connector.instance().store_points(points)


def main(args=None):

    rclpy.init(args=args)

    middleware = Middleware()

    try:
        rclpy.spin(middleware)  # Run Middleware continuously
    except KeyboardInterrupt:
        pass

    # Close the database connection
    db_instance = SpatialData_DB_Connector.instance()
    del db_instance

    # Destroy nodes and shutdown ROS
    middleware.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
