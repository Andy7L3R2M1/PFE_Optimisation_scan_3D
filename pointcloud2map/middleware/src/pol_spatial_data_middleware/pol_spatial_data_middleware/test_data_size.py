#!/usr/bin/env python

import rclpy
from pympler.asizeof import asizeof
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from visualization_msgs.msg import MarkerArray
from pol_msgs.msg import SpatialData


class DataSubscriber(Node):
    def __init__(self):
        super().__init__("data_subscriber")
        self.depth_image_subscriber = self.create_subscription(
            Image, "/DT1/depth/depth/image_raw", self.depth_image_callback, 1
        )
        self.color_image_subscriber = self.create_subscription(
            Image, "/DT1/color/image_raw", self.color_image_callback, 1
        )
        self.pointcloud_subscriber = self.create_subscription(
            PointCloud2, "/DT1/depth/points", self.pointcloud_callback, 1
        )
        self.marker_array_subscriber = self.create_subscription(
            MarkerArray, "/DT1/occupied_cells_vis_array", self.marker_array_callback, 1
        )
        self.spatial_data_subscriber = self.create_subscription(
            SpatialData, "/observer_1/spatial_data", self.spatial_data_callback, 1
        )

    def depth_image_callback(self, msg):
        self.get_logger().info("Size of depth image: " + str(asizeof(msg)))
        self.destroy_subscription(self.depth_image_subscriber)

    def color_image_callback(self, msg):
        self.get_logger().info("Size of color image: " + str(asizeof(msg)))
        self.destroy_subscription(self.color_image_subscriber)

    def pointcloud_callback(self, msg):
        self.get_logger().info("Size of pointcloud: " + str(asizeof(msg)))
        self.destroy_subscription(self.pointcloud_subscriber)

    def marker_array_callback(self, msg):
        nb_points = 0
        sum_size_points = 0
        for marker in msg.markers:
            for point in marker.points:
                nb_points += 1
                sum_size_points += asizeof(point)
        self.get_logger().info(f"Number of points: {nb_points}")
        self.get_logger().info(f"Average size of a point: {sum_size_points/nb_points}")
        self.get_logger().info(f"Total size of points: {sum_size_points}")
        self.get_logger().info("Size of marker array: " + str(asizeof(msg)))
        self.destroy_subscription(self.marker_array_subscriber)

    def spatial_data_callback(self, msg):
        self.get_logger().info("Size of spatial data: " + str(asizeof(msg)))
        self.destroy_subscription(self.spatial_data_subscriber)


def main(args=None):
    rclpy.init(args=args)

    data_subscriber = DataSubscriber()

    try:
        rclpy.spin(data_subscriber)  # Run DataSubscriber continuously
    except KeyboardInterrupt:
        pass

    # Destroy nodes and shutdown ROS
    data_subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
