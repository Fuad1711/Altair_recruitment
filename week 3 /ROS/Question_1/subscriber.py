#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RoverSubscriber(Node):
    def __init__(self):
        super().__init__('rover_subscriber')

        self.subscription = self.create_subscription(String, 'rover_topic', self.listener_callback, 10)
        
        self.get_logger().info('Subscriber node has been started and is listening.')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = RoverSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
