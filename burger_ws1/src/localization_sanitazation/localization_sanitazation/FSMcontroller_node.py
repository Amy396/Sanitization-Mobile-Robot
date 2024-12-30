'''
ROS2 Humble node to control a TurtleBot3 using the velocity topic.
'''

import rclpy
from rclpy.node import Node
import numpy as np

# Import /cmd_vel topic message type
from geometry_msgs.msg import Twist

# Import Laser scanner message type
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

# FSM State definition
TB3_MOVING = 0
TB3_ROTATING = 1
TB3_RECOVERY = 2

# Minimum safe distance for obstacle avoidance
MIN_SAFE_DISTANCE = 1.0  # in meters

class FSMController(Node):
    def __init__(self):
        super().__init__('FSMcontroller')

        # Create Publisher and Subscriber
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.laser_subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.localization_complete_subscriber = self.create_subscription(Bool, 'localization_complete', self.bool_callback, 10)

        # Initialize states and variables
        self.tb3_state = TB3_MOVING
        self.laser_data = None
        self.localization_complete = False

    def move(self):
        '''Function to move the TurtleBot forward.'''
        vel_msg = Twist()
        vel_msg.linear.x = 0.15    #m/s
        vel_msg.angular.z = 0.05   #radian/s = 0.05radian/s is approximately 2.86 degrees per second
        self.velocity_publisher.publish(vel_msg)
        # self.node.get_logger().info(f'Publishing new velocity X: {vel_msg.linear.x} Z: {vel_msg.angular.z}')

    def rotate(self):
        '''Function to rotate the TurtleBot.'''
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.1
        self.velocity_publisher.publish(vel_msg)
        # self.node.get_logger().info(f'Publishing new velocity X: {vel_msg.linear.x} Z: {vel_msg.angular.z}')

    def stop(self):
        '''Function to stop the TurtleBot.'''
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(vel_msg)
        # self.node.get_logger().info(f'Publishing new velocity X: {vel_msg.linear.x} Z: {vel_msg.angular.z}')

    def laser_callback(self, msg):
        '''Callback function to store laser scanner data.'''
        self.laser_data = msg

    def check_collision(self):
        '''Check for potential collisions using laser scan data.'''
        if self.laser_data is None:
            self.get_logger().info("No laser data received")
            return False

        front_data = self.laser_data.ranges[-25:] + self.laser_data.ranges[:25]
        for distance in front_data:
            if distance < MIN_SAFE_DISTANCE:
                return True
        return False

    def bool_callback(self, msg):
        '''Callback function to check if localization is complete.'''
        self.localization_complete = msg.data
        if self.localization_complete:
            self.get_logger().info("Localization complete, shutting down the node.")
            self.shutdown_node()

    def shutdown_node(self):
        '''Shut down the node gracefully.'''
        self.stop()
        self.destroy_node()
        rclpy.shutdown()

    def control_loop(self):
        '''Main control loop to manage the robot's behavior using a finite state machine.'''
        while rclpy.ok():
            rclpy.spin_once(self)

            if self.tb3_state == TB3_MOVING:
                self.move()
                if self.check_collision():
                    self.tb3_state = TB3_ROTATING

            elif self.tb3_state == TB3_ROTATING:
                self.rotate()
                if not self.check_collision():
                    self.tb3_state = TB3_MOVING

            elif self.tb3_state == TB3_RECOVERY:
                self.get_logger().info("Recovery state is not yet implemented.")
                continue


def main(args=None):
    '''Main function to initialize the node and start the control loop.'''
    rclpy.init(args=args)
    controller = FSMController()
    controller.control_loop()
    

if __name__ == '__main__':
    main()