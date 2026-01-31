#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class URDFPublisher(Node):
    def __init__(self, urdf_path):
        super().__init__('urdf_publisher')
        self.publisher_ = self.create_publisher(String, '/tita4267305/robot_description', 10)

        # Read URDF file
        with open(urdf_path, 'r') as f:
            self.urdf_content = f.read()

        # Publish once after a short delay
        self.timer = self.create_timer(1.0, self.publish_urdf)  # 1 second timer
        self.published = False

    def publish_urdf(self):
        if not self.published:
            msg = String()
            msg.data = self.urdf_content
            self.publisher_.publish(msg)
            self.get_logger().info('URDF published to /tita4267305/robot_description')
            self.published = True
            rclpy.shutdown()  # exit after publishing

def main(args=None):
    rclpy.init(args=args)
    # Replace 'file.urdf' with your test URDF file path
    urdf_publisher = URDFPublisher('/home/emiliano/Desktop/ros2_ws/src/tita_controller/tita_description/tita.urdf')
    rclpy.spin(urdf_publisher)

if __name__ == '__main__':
    main()
