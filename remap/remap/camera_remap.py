#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2

class TopicRemapper(Node):
    def __init__(self):
        super().__init__('topic_remapper')

        # Declare parameters so they can be overridden at runtime
        self.declare_parameter('input_image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('output_image_topic', '/camera/color/image_raw')

        self.declare_parameter('input_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('output_info_topic', '/camera/color/camera_info')

        self.declare_parameter('input_depth_topic', '/camera/camera/depth/color/points')
        self.declare_parameter('output_depth_topic', '/camera/depth/color/points')
        
        input_image_topic = self.get_parameter('input_image_topic').get_parameter_value().string_value
        output_image_topic = self.get_parameter('output_image_topic').get_parameter_value().string_value

        input_info_topic = self.get_parameter('input_info_topic').get_parameter_value().string_value
        output_info_topic = self.get_parameter('output_info_topic').get_parameter_value().string_value

        input_depth_topic = self.get_parameter('input_depth_topic').get_parameter_value().string_value
        output_depth_topic = self.get_parameter('output_depth_topic').get_parameter_value().string_value

        # Create a image subscription for the input topic
        self.subscription = self.create_subscription(
            Image,
            input_image_topic,
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(Image, output_image_topic, 10)
        
        # Create a camera info subscription for the input topic
        self.info_subscription = self.create_subscription(
            CameraInfo,
            input_info_topic,
            self.info_callback,
            10
        )
        self.info_publisher = self.create_publisher(CameraInfo, output_info_topic, 10)
        
        # Create a depth subscription for the input topic
        self.depth_subscription = self.create_subscription(
            PointCloud2,
            input_depth_topic,
            self.depth_callback,
            10
        )
        self.depth_publisher = self.create_publisher(PointCloud2, output_depth_topic, 10)
        
        # info
        self.get_logger().info(f"Remapping '{input_image_topic}' to '{output_image_topic}'")
        self.get_logger().info(f"Remapping '{input_info_topic}' to '{output_info_topic}'")
        self.get_logger().info(f"Remapping '{input_depth_topic}' to '{output_depth_topic}'")

    def image_callback(self, msg: Image):
        self.publisher.publish(msg)
    
    def info_callback(self, msg: CameraInfo):
        self.info_publisher.publish(msg)
    
    def depth_callback(self, msg: PointCloud2):
        self.depth_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TopicRemapper()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

