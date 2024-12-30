'''
ROS2 Humble node to control a turtlebot3 using the velocity topic 

'''

import rclpy
import time 
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

import numpy as np   

TB3_INIT = 3
TB3_MOVING = 0
TB3_ROTATING = 0.5
TB3_STOPPED = 2

COLLISION_THRESHOLD = 0.7 # m
FRONT_RANGE   = 20
ROTATION_VEL  = 2 * np.pi / 180 # 5 degrees/seconds in radians
LOOP_RATE     = 30 # Hz, common for high level control
MAX_ROT_STEPS =  (360 * 5) * 30 * 2


class FSMController(Node):



   def __init__(self) -> None:
      super().__init__('FSM_Controller')
      # Initialize Node
     
      # Create Publisher
      self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

      # Add subscriber to laser scanner
      self.subscriber = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
      self.laser_data = None

      # Create Rate to specify the frequency of the loop
      #self.timer = self.create_timer(1/LOOP_RATE, self.control_loop) #Hz
      self.rate = self.create_rate(LOOP_RATE)


      self.tb3_state = TB3_INIT
      self.rotation_step_counter = 0


      pass

   def laser_callback(self, msg):
    # Get ranges between [-FRONT_RANGE, FRONT_RANGE]
    self.laser_data = msg.ranges[-FRONT_RANGE:] + msg.ranges[:FRONT_RANGE]
  


   def check_init_completed(self):
      # Just checking if the a new data is arrived between starting the node and the first loop
      # As alternative, use Service/Server to activate the movement
      
      if self.laser_data == None :
         return False
      else:
         return True
      


   def move(self):
        # Create an empty Twist message
        msg = Twist()

        # Set the linear velocity
        msg.linear.x = 0.22
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        # Set the angular velocity
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        # Publish the message
        self.publisher.publish(msg)
      
   def rotate(self):
        msg = Twist()

        # Set the linear velocity
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        # Set the angular velocity
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.7

        # Publish the message
        self.publisher.publish(msg)

   def stop(self):
        msg = Twist()

        # Set the linear velocity
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        # Set the angular velocity
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        # Publish the message
        self.publisher.publish(msg)

   def check_collision(self):
      '''
      Add function that control if it's in collission.
      Use the laser scanner information from the turtlebot.
      '''
      
      if any(i < COLLISION_THRESHOLD for i in self.laser_data):
         return True
      else :
         return False
   


  
    
    # Run your navigation loop with velocity control
    

    
   def control_loop(self):
      # enter state machine
      # Iterate with logger at given rate
      self.get_logger().info(f'Entering state machine: {self.tb3_state}')

      duration = 20  # seconds
      end_time = time.time() + duration  # Calculate the end time
      
      while time.time() < end_time and  rclpy.ok():
         rclpy.spin_once(self)

         # TB3 STARTING STATE
         if self.tb3_state == TB3_INIT:
            if self.check_init_completed():
               print('Moving Forward')
               self.tb3_state = TB3_MOVING
               
         # TB3 moving forward
         elif self.tb3_state == TB3_MOVING:
            self.move()

            if self.check_collision():
               print('Moving Rotating')
               self.tb3_state = TB3_ROTATING
               self.rotation_step_counter = 0
         
         # TB3 is rotating
         elif self.tb3_state == TB3_ROTATING:
            self.rotate()
            if not self.check_collision():
               print('Moving Forward')
               self.tb3_state = TB3_MOVING
         
            if self.rotation_step_counter > MAX_ROT_STEPS :
               print('Exciding allowed steps')
               self.tb3_state = TB3_STOPPED
            else :
               self.rotation_step_counter += 1

         # TB3 is stopped and is not moving
         elif self.tb3_state == TB3_STOPPED:
            print('Robot Error ....')
            self.stop()
         
         

def main(args=None):
   rclpy.init(args=args)

   controller = FSMController()

   controller.control_loop()
  
   controller.destroy_node()
   rclpy.shutdown()



if __name__ == '__main__':
   main()