'''
ROS2 Humble node to control a turtlebot3 using the velocity topic 
'''

import rclpy
from geometry_msgs.msg import Twist
import random
import time
from rclpy.node import Node

class Rotation(Node):
    ''' Node to control Turtlebot3's roation velocity. '''

    def __init__(self) -> None:
        # Initialize Node
        self.node = rclpy.create_node('rotation')
        
        # Create Publisher
        self.publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create Timer
        self.timer = self.node.create_timer(2.0, self.publish_vel)

    def publish_vel(self):
        ''' Publishes a new random velocity. '''
        print('Rotating')
        # Create Twist Message
        msg = Twist()
        
        # Generate Random velocity
        # x_lin = random.uniform(-0.2, 0.2)
        # z_rot = random.uniform(-0.2, 0.2)
        x_lin = 0.0
        z_rot = -0.5
        # Assign Random velocity to Twist Message
        msg.linear.x = x_lin
        msg.angular.z = z_rot
        
        # Publish Twist Message
        self.publisher.publish(msg)
        
        # Print Twist Message
        self.node.get_logger().info(f'Publishing: vel: {msg.linear.x}, rot: {msg.angular.z}')

    def control_loop(self):
        duration = 12  # seconds
        end_time = time.time() + duration  # Calculate the end time
        print("Rotating")
        while time.time() < end_time and  rclpy.ok():
            rclpy.spin_once(self.node)

        msg = Twist()
        x_lin = 0.0
        z_rot = 0.0
        msg.linear.x = x_lin
        msg.angular.z = z_rot
        self.publisher.publish(msg)
        # Publish Twist Message

def main(args=None):
    rclpy.init()
    print('Starting Rotation node')
    rotation = Rotation()
    rotation.control_loop()  # Corrected line
    rotation.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
