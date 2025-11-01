#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
import threading
import math
import time
import sys

class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer_ = self.create_timer(0.05, self.publish_command) 
        
        self.get_logger().info("Turtle Controller Node Started.")
        self.current_mode = 'stop'
        self.twist_msg = Twist() 
        
        self.square_state = 'forward'
        self.square_state_start_time = self.get_clock().now()
        self.square_move_duration = Duration(seconds=1.5)
        self.square_turn_duration = Duration(seconds=1.0)

        self.spiral_linear_vel = 0.5
        
        self.input_thread = threading.Thread(target=self.get_user_input, daemon=True)
        self.input_thread.start()


    def get_user_input(self):

        while rclpy.ok():
            key = input("Enter command (A, B, C): ").strip().upper()
            
            if key == 'A':
                self.set_mode('circle')
            elif key == 'B':
                self.set_mode('square')
            elif key == 'C':
                self.set_mode('spiral')
            else:
                print("Invalid command.")

    def set_mode(self, mode):
        if self.current_mode == mode:
            return
            
        self.get_logger().info(f"Changing mode to: {mode}")
        self.current_mode = mode
        
        if mode == 'square':

            self.square_state = 'forward'
            self.square_state_start_time = self.get_clock().now()
        
        elif mode == 'spiral':

            self.spiral_linear_vel = 0.5
            
    def publish_command(self):
        self.twist_msg = Twist()
        now = self.get_clock().now()
        
        if self.current_mode == 'stop':
            pass
            
        elif self.current_mode == 'circle':
            self.twist_msg.linear.x = 2.0
            self.twist_msg.angular.z = 1.0
            
        elif self.current_mode == 'square':
            elapsed_time = now - self.square_state_start_time
            
            if self.square_state == 'forward':
                self.twist_msg.linear.x = 2.0
                if elapsed_time >= self.square_move_duration:
                    self.square_state = 'turn'
                    self.square_state_start_time = now
                    
            elif self.square_state == 'turn':
                self.twist_msg.angular.z = math.pi / 2.0
                if elapsed_time >= self.square_turn_duration:
                    self.square_state = 'forward'
                    self.square_state_start_time = now
        
        elif self.current_mode == 'spiral':
            self.twist_msg.linear.x = self.spiral_linear_vel
            self.twist_msg.angular.z = 2.0
            
            self.spiral_linear_vel += 0.01
        self.publisher_.publish(self.twist_msg)

def main(args=None):
    rclpy.init(args=args)
    
    controller_node = TurtleControllerNode()
    rclpy.spin(controller_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


