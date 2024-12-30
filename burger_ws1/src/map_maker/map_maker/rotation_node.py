'''
ROS2 Humble node to control a TurtleBot3's rotation using the velocity topic.
'''

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
import time

class Rotation(Node):
    '''Node to control TurtleBot3's rotational velocity.'''

    def __init__(self):
        super().__init__('rotation')  # Initialize the node with the name 'rotation'
        
        # Create a publisher for the 'cmd_vel' topic
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create a timer to call the `publish_vel` function every 2 seconds
        self.timer = self.create_timer(2.0, self.publish_vel)
    ######################
    #Publishes a fixed rotation velocity.
    ######################
    def publish_vel(self):
        
        self.get_logger().info('Rotating...')

        # Create a Twist message
        msg = Twist()
        msg.linear.x = 0.0  # No forward/backward movement
        msg.angular.z = -0.5  # Rotate counterclockwise

        # Publish the velocity message
        self.publisher.publish(msg)
        
        # Log the published velocity
        self.get_logger().info(f'Published: linear.x = {msg.linear.x}, angular.z = {msg.angular.z}')

    #######################
    #Controls the robot's rotation for a specified duration.
    #######################
    def control_loop(self):
        
        duration = 12  # Duration of rotation in seconds
        end_time = time.time() + duration  # Calculate the end time

        # Rotate the robot for the specified duration
        while time.time() < end_time and rclpy.ok():
            rclpy.spin_once(self)

        # Stop the robot after the duration
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info('Rotation complete. Robot stopped.')

def main(args=None):
    
    rclpy.init(args=args)  # Initialize ROS2
    rotation = Rotation()  # Create an instance of the Rotation class
    rotation.control_loop()  # Execute the rotation logic
    rotation.destroy_node()  # Clean up resources
    rclpy.shutdown()  # Shut down ROS2

if __name__ == '__main__':
    main()
