
import time
import numpy as np
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped,PoseArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys
from my_robot_controller.velocity_control_FSM import velocityControllerFSM
from my_robot_controller.velocity_control import VelocityController


import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile
from rclpy.time import Time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
  



from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid 
import heapq , math , random , yaml
import scipy.interpolate as si
import threading , time





####################################################################
########### Navigation Algoritm based on AMCL ######################
####################################################################
class BasicNavigator(Node):
    def __init__(self):
        super().__init__('basic_navigator')
        self.initial_pose = Pose()
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        self.latest_odom_pose = None  # Variable to store the latest odometry pose
        self.NavGoal = Pose()
        self.path_msg = PoseArray()
        self.message_received = False
        self.initial_pose_received = False
        self.path_received = False
        #self.path_san_received = False

        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        self.initial_pose_received = False
        self.nav_through_poses_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.model_pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amclPoseCallback, amcl_pose_qos)
        self.subscription_f = self.create_subscription(Pose, 'frontier', self._FrontierCallback,10)
        # self.subscription_sanitize = self.create_subscription(PoseArray, 'sanitization_poses', self._SanitizeCallback,10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.suscription_path = self.create_subscription(PoseArray, 'path',self._PathCallback, 10)
        self.target_status = self.create_publisher(String, 'status', 10)
       
        self.odom_pose_sub = self.create_subscription(Odometry, 'odom', self._odomPoseCallback, QoSProfile(depth=1))
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        


    def _createTwistMsg(self, linear=[0.0, 0.0, 0.0], angular=[0.0, 0.0, 0.0]):
        """
        Create a Twist message.

        Args:
            linear (list): List of linear velocities [x, y, z].
            angular (list): List of angular velocities [x, y, z].

        Returns:
            geometry_msgs.msg.Twist: Twist message.
        """
        twist_msg = Twist()
        twist_msg.linear.x = linear[0]
        twist_msg.linear.y = linear[1]
        twist_msg.linear.z = linear[2]
        twist_msg.angular.x = angular[0]
        twist_msg.angular.y = angular[1]
        twist_msg.angular.z = angular[2]
        return twist_msg
    
    def moveRobot(self, linear_velocity):
        """
        Move the robot with the specified linear velocity.
        """
        twist_msg = self._createTwistMsg(linear=[linear_velocity, 0.0, 0.0], angular=[0.0, 0.0, 0.0])
        self.cmd_vel_pub.publish(twist_msg)
    
    def stopRobot(self):
        """
        Stop the robot.
        """
        twist_msg = self._createTwistMsg(linear=[0.0, 0.0, 0.0], angular=[0.0, 0.0, 0.0])
        self.cmd_vel_pub.publish(twist_msg)

    def turnRobot(self, angular_velocity):
        """
        Turn the robot.

        Args:
            angular_velocity (float): Angular velocity in rad/s.
        """
        twist_msg = self._createTwistMsg(angular=[0.0, 0.0, angular_velocity])
        self.cmd_vel_pub.publish(twist_msg)

 

    def setInitialPose(self, initial_pose):
        self.initial_pose_received = False
        self.initial_pose = initial_pose
        self._setInitialPose()


    def goThroughPoses(self, poses):
        # Sends a `NavToPose` action request and waits for completion
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses

        self.info('Navigating with ' + str(len(poses)) + ' goals.' + '...')
        send_goal_future = self.nav_through_poses_client.send_goal_async(goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal with ' + str(len(poses)) + ' poses was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

   

    
    def goToPose(self, pose):
        # Sends a `NavToPose` action request and waits for completion
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                  str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                       str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def cancelNav(self):
        self.info('Canceling current goal.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isNavComplete(self):
        if not self.result_future:
            # task was canceled or completed
            return True

        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.info('Goal with failed with status code: {0}'.format(self.status))
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.info('Goal succeeded!')
        return True

    def getFeedback(self):
        return self.feedback

    def getResult(self):
        return self.status

    def waitUntilNav2Active(self):
        # self._waitForNodeToActivate('amcl')
        # self._waitForInitialPose()
        self._waitForNodeToActivate('bt_navigator')
        self.info('Nav2 is ready for use!')
        return

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug('Waiting for ' + node_name + ' to become active..')
        node_service = node_name + '/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(node_service + ' service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while (state != 'active'):
            self.debug('Getting ' + node_name + ' state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug('Result of get_state: %s' % state)
            time.sleep(2)
        return

    def _waitForInitialPose(self):
        while not self.initial_pose_received:
            self.info('Setting initial pose')
            self._setInitialPose()
            self.info('Waiting for amcl_pose to be received')
            rclpy.spin_once(self, timeout_sec=1)
        return

    def _amclPoseCallback(self, msg):
        self.initial_pose_received = True
        return

    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        return
        
    def _FrontierCallback(self,msg):
        self.NavGoal.position.x = msg.position.x
        self.NavGoal.position.y = msg.position.y
        self.message_received = True
        return
    
    def _PathCallback(self,msg):
        self.path_msg = msg
        self.path_received = True
        return
 
       
    def _setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        return
        
    def _odomPoseCallback(self, msg):
        # Update the latest odometry pose
        self.initial_pose_received = True
        self.latest_odom_pose = msg.pose.pose
        

    def info(self, msg):
        self.get_logger().info(msg)
        return
    
    def debug(self, msg):
        self.get_logger().debug(msg)
        return
    def warn(self, msg):
        self.get_logger().warn(msg)
        return
    
    
 ############################################################
 ########################  MAIN CODE  #######################
 ############################################################
def main(argv=sys.argv[1:]):


    rclpy.init()
    navigator = BasicNavigator()

    velocity_controllerFSM = velocityControllerFSM()
    rotation = VelocityController()

    # duration = 1000  # seconds
    # end_time = time.time() + duration  # Calculate the end time

    ######################################
    #Step0: send the intial Pose for amcl
    ######################################
    # while not navigator.initial_pose_received:
    #         navigator.get_logger().info(f"wating for Initial Pose")
    #         rclpy.spin_once(navigator, timeout_sec=1.0)

    # initial_pose = Pose()
    # initial_pose.position.x =  navigator.latest_odom_pose.position.x
    # initial_pose.position.y = navigator.latest_odom_pose.position.y
    # initial_pose.orientation.z = navigator.latest_odom_pose.position.z
    # #Orientation Values
    # initial_pose.orientation.x = navigator.latest_odom_pose.orientation.x
    # initial_pose.orientation.y = navigator.latest_odom_pose.orientation.y
    # initial_pose.orientation.z = navigator.latest_odom_pose.orientation.z
    # initial_pose.orientation.w = navigator.latest_odom_pose.orientation.w
    # print('Sending initial pose...')
    # navigator.setInitialPose(initial_pose)
    # input('Pose ok, enter to continue')

    #Wait for navigation to fully activate
    navigator.waitUntilNav2Active()
    input('Navigation2 ok, enter to continue')
    
    ################################
    # step1: Initial random movement
    ################################
    # duration = 100
    # end_time = time.time()+duration
    # while time.time() < end_time and rclpy.ok():
    #     rclpy.spin_once(navigator, timeout_sec=0.1)
    #     velocity_controllerFSM.control_loop()
    #     # Stop the velocity controller after the specified duration
    # velocity_controllerFSM.stop()
    # velocity_controllerFSM.control_loop()
    # velocity_controllerFSM.stop()



    while True:
 
        ###############################################################
        # step1: Get Goal
        ###############################################################
        while not navigator.message_received:
            navigator.get_logger().info(f"wating for the navigation goal")
            rclpy.spin_once(navigator, timeout_sec=1.0)



        ########################
        # step2: Go to the Goal
        ########################
        # Go to the last pose of the returned path
        # if navigator.path_msg.poses:
        #     last_pose = navigator.path_msg.poses[-1]
            
        last_pose = navigator.NavGoal
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x =    last_pose.position.x
        goal_pose.pose.position.y =    last_pose.position.y
        # Set orientation to represent no rotation
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        navigator.goToPose(goal_pose)
   
        print('Navigator go to pose')
        i = 0
        while not navigator.isNavComplete():
           
            # while not navigator.message_received:
            #     navigator.get_logger().info(f"wating for the navigation goal")
            #     rclpy.spin_once(navigator, timeout_sec=1.0)
                
            # Do something with the feedback
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    rclpy.duration.Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')
                # Some navigation timeout to demo cancellation
                if rclpy.duration.Duration.from_msg(feedback.navigation_time) > rclpy.duration.Duration(seconds = 100.0):
                    # Cancel the current goal
                    navigator.cancelNav()
                    

        
        # Do something depending on the return code
        result = navigator.getResult()
        if result == GoalStatus.STATUS_SUCCEEDED:
            print('Goal succeeded!')
            velocity_controllerFSM.control_loop()
            velocity_controllerFSM.stop() 
            # rotation.control_loop()
            navigator.message_received = False

        elif result == GoalStatus.STATUS_CANCELED:
            print('Goal was canceled!')
            status_msg = String()
            status_msg.data = "Goal Rejected"
            navigator.target_status.publish(status_msg)
            velocity_controllerFSM.control_loop()
            velocity_controllerFSM.stop() 
            navigator.message_received = False

        elif result == GoalStatus.STATUS_ABORTED:
            print('Goal failed!')
            status_msg = String()
            status_msg.data = "Goal Rejected"
            navigator.target_status.publish(status_msg)
            velocity_controllerFSM.control_loop()
            velocity_controllerFSM.stop()
            navigator.message_received = False

        else:
            print('Goal has an invalid return status!')
        rclpy.spin_once(navigator)
     
if __name__ == '__main__':
    main()