#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class URDFSaver(Node):
    def __init__(self):
        super().__init__('urdf_saver')
        # Subscribe to the topic
        self.subscription = self.create_subscription(
            String,
            '/tita4267305/robot_description',
            self.urdf_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def urdf_callback(self, msg):
        urdf_content = msg.data
        # Save URDF content to a file
        with open('file.urdf', 'w') as f:
            f.write(urdf_content)
        self.get_logger().info('URDF saved to file.urdf')
        rclpy.shutdown()  # stop after saving

def main(args=None):
    rclpy.init(args=args)
    urdf_saver = URDFSaver()
    rclpy.spin(urdf_saver)

if __name__ == '__main__':
    main()



# to use: 
# chmod +x save_urdf.py
# ros2 run <your_package> save_urdf.py
