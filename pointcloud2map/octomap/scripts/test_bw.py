#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from pympler.asizeof import asizeof
from collections import defaultdict
import time

class DynamicTopicSubscriber(Node):
    def __init__(self):
        super().__init__("dynamic_topic_subscriber")
        self.topic_subscriptions = {}  # To store subscriptions
        self.message_sizes = defaultdict(int)  # Track message sizes
        self.message_counts = defaultdict(int)  # Track message counts
        self.start_times = defaultdict(lambda: time.time())  # Start times for each topic

        # Periodically discover topics and print bandwidth
        self.create_timer(1.0, self.discover_topics)
        self.create_timer(1.0, self.print_bandwidth)

    def discover_topics(self):
        topic_names_and_types = self.get_topic_names_and_types()
        for item in topic_names_and_types:
            if len(item) != 2:
                self.get_logger().warn(f"Skipping topic due to unexpected format: {item}")
                continue

            topic_name, topic_types = item
            if not topic_types:
                self.get_logger().warn(f"No types found for topic {topic_name}, skipping...")
                continue

            if topic_name in self.topic_subscriptions:
                continue

            topic_type = topic_types[0].replace("/msg/", ".")  # Convertir 'pkg/msg/Type' en 'pkg.msg.Type'

            if "." not in topic_type:
                self.get_logger().error(f"Invalid message type format: {topic_type}")
                continue

            try:
                module_name, message_name = topic_type.rsplit(".", 1)
                msg_module = __import__(module_name, fromlist=[message_name])
                msg_class = getattr(msg_module, message_name)

                qos_profile = QoSProfile(depth=10, durability=QoSDurabilityPolicy.VOLATILE)
                subscription = self.create_subscription(
                    msg_class,
                    topic_name,
                    lambda msg, t=topic_name: self.message_callback(msg, t),
                    qos_profile
                )
                self.topic_subscriptions[topic_name] = subscription
                self.get_logger().info(f"Subscribed to topic: {topic_name} ({topic_type})")

            except Exception as e:
                self.get_logger().error(f"Failed to subscribe to topic {topic_name}: {e}")

    def message_callback(self, msg, topic_name):
        msg_size = asizeof(msg)
        self.message_sizes[topic_name] += msg_size
        self.message_counts[topic_name] += 1

    def print_bandwidth(self):
        for topic_name in self.message_sizes.keys():
            elapsed_time = time.time() - self.start_times[topic_name]
            if elapsed_time > 0:
                total_size = self.message_sizes[topic_name]
                total_messages = self.message_counts[topic_name]
                bandwidth = total_size / elapsed_time if total_size > 0 else 0
                avg_msg_size = total_size / total_messages if total_messages > 0 else 0

                self.get_logger().info(
                    f"Topic: {topic_name}, Bandwidth: {bandwidth:.2f} B/s, "
                    f"Avg Msg Size: {avg_msg_size:.2f} B, Msg Count: {total_messages}"
                )
                self.message_sizes[topic_name] = 0
                self.message_counts[topic_name] = 0
                self.start_times[topic_name] = time.time()

    def cleanup_stale_topics(self):
        active_topics = set(name for name, _ in self.get_topic_names_and_types())
        for topic in list(self.topic_subscriptions.keys()):
            if topic not in active_topics:
                self.get_logger().info(f"Unsubscribing from stale topic: {topic}")
                self.destroy_subscription(self.topic_subscriptions.pop(topic))

def main(args=None):
    rclpy.init(args=args)
    node = DynamicTopicSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
