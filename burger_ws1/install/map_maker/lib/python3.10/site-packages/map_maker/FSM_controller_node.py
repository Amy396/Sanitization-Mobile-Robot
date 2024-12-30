'''
ROS2 Humble node to control a TurtleBot3 using the velocity topic.

Initialization:
    Ensures laser data is available before transitioning to the moving state.

Collision Avoidance:
    Detects obstacles using laser scans and transitions to a rotating state.

FSM Logic:
    Cycles between moving, rotating, and stopping states based on sensor feedback.

Robot Control:
    Uses Twist messages to send velocity commands to the robot.
'''

import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

# FSM States
TB3_INIT = 0
TB3_MOVING = 1
TB3_ROTATING = 2
TB3_STOPPED = 3

# Constants
COLLISION_THRESHOLD = 0.7               # Minimum distance (meters) to an obstacle before detecting a collision.
FRONT_RANGE = 20                        # Angular range for checking obstacles in front.
ROTATION_VEL = 2 * np.pi / 180          # Angular velocity for rotation (~0.087 rad/s or ~5°/s).
LOOP_RATE = 30                          # Loop frequency (30 Hz).
MAX_ROT_STEPS = (360 * 5) * 30 * 2      # Maximum steps for rotation before stopping (~216,000 steps for 360° rotation).

class FSMController(Node):

    def __init__(self) -> None:
        super().__init__('FSM_Controller')
        
        # Initialize publisher and subscriber
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        
        # Variables
        self.laser_data = None
        self.tb3_state = TB3_INIT
        self.rotation_step_counter = 0

    #######################
    # Processes laser scan data from the 'scan' topic.
    #######################
    def laser_callback(self, msg):
        self.laser_data = msg.ranges[-FRONT_RANGE:] + msg.ranges[:FRONT_RANGE]

    ########################
    # Checks if laser scan data is available to confirm initialization is complete.
    ########################
    def check_init_completed(self):
        return self.laser_data is not None

    #######################
    # Sends a command to move the robot forward.
    #######################
    def move(self):
        msg = Twist()
        msg.linear.x = 0.22
        self.publisher.publish(msg)

    #######################
    # Sends a command to rotate the robot.
    #######################
    def rotate(self):
        msg = Twist()
        msg.angular.z = ROTATION_VEL
        self.publisher.publish(msg)

    #######################
    # Stops the robot by sending zero velocities.
    #######################
    def stop(self):
        msg = Twist()
        self.publisher.publish(msg)

    #########################
    # Checks for potential collisions based on laser scan data.
    #########################
    def check_collision(self):
        return any(i < COLLISION_THRESHOLD for i in self.laser_data)

    #########################
    # FSM control loop for managing the robot's behavior.
    #########################
    def control_loop(self):
        self.get_logger().info(f'Entering state machine: {self.tb3_state}')
        
        duration = 20  # seconds
        end_time = time.time() + duration
        
        while time.time() < end_time and rclpy.ok():
            rclpy.spin_once(self)

            # FSM Logic
            if self.tb3_state == TB3_INIT:
                if self.check_init_completed():
                    print('Initialization complete. Moving forward...')
                    self.tb3_state = TB3_MOVING

            elif self.tb3_state == TB3_MOVING:
                self.move()
                if self.check_collision():
                    print('Obstacle detected. Rotating...')
                    self.tb3_state = TB3_ROTATING
                    self.rotation_step_counter = 0

            elif self.tb3_state == TB3_ROTATING:
                self.rotate()
                if not self.check_collision():
                    print('Path cleared. Moving forward...')
                    self.tb3_state = TB3_MOVING

                if self.rotation_step_counter > MAX_ROT_STEPS:
                    print('Exceeded maximum rotation steps. Stopping...')
                    self.tb3_state = TB3_STOPPED
                else:
                    self.rotation_step_counter += 1

            elif self.tb3_state == TB3_STOPPED:
                print('Robot stopped.')
                self.stop()

def main(args=None):
    rclpy.init(args=args)
    controller = FSMController()
    controller.control_loop()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
