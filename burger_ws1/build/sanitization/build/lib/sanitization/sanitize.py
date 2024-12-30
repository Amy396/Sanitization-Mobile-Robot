
import time
import numpy as np
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import sys



import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile
from rclpy.time import Time
#from my_robot_controller.velocity_control_FSM import velocityController



#Reference table of the corners of the big-house rooms
""" corner1:= top-left
    corner2:= top-right
    corner3:= bottom-right
    corner4:= bottom-left
"""
room_reference_table = {
        'room1': {
            'corner1': {'x': -7.2, 'y': -4.5, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'corner2': {'x': 6.6, 'y': -6.0, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'corner3': {'x': 6.6, 'y': -7.7, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'corner4': {'x': -7.6, 'y': -7.2, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'center': None  # To be computed
        },
       
        'room2': {
            'corner1': {'x': -4.8, 'y': -0.3, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'corner2': {'x': 4.5, 'y': -0.8, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'corner3': {'x': 4.5, 'y': -3.7, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'corner4': {'x': -4.8, 'y': -3.6, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'center': None  # To be computed
        },
        'room3': {
            'corner1': {'x': 3.5, 'y': 4.5, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'corner2': {'x': 6.5, 'y': 4.5, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'corner3': {'x': 6.5, 'y': 0.5, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'corner4': {'x': 3.5, 'y': 0.5, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'center': None  # To be computed
        },
        'room4': {
            'corner1': {'x': 5.0, 'y': -1.0, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'corner2': {'x': 7.0, 'y': -1.0, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'corner3': {'x': 7.0, 'y': -5.0, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'corner4': {'x': 5.0, 'y': -5.0, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'center': None  # To be computed
        },
        'room5': {
            'corner1': {'x': 0.4, 'y': 4.6, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'corner2': {'x': 2.0, 'y': 4.8, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'corner3': {'x': 1.6, 'y': 1.2, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'corner4': {'x': 0.0, 'y': 1.2, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'center': None  # To be computed
        },
        'room6': {
            'corner1': {'x': -4.5, 'y':4.5, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'corner2': {'x': -0.6, 'y': 5.0, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'corner3': {'x': -0.6, 'y': 0.0, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'corner4': {'x': -4.9, 'y': 0.2, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'center': None  # To be computed
        },
        'room7': {
            'corner1': {'x': -7.0, 'y': 4.5, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'corner2': {'x': -5.5, 'y': 5.0, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'corner3': {'x': -5.5, 'y': 1.5, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'corner4': {'x': -7.0, 'y': 1.5, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'center': None  # To be computed
        },
        'room8': {
            'corner1': {'x': -7.0, 'y': 0.5, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'corner2': {'x': -5.5, 'y': 0.5, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'corner3': {'x': -5.5, 'y': -3.7, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'corner4': {'x': -7.0, 'y': -3.7, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'center': None  # To be computed
        },
        
    }


#A list of rooms that has been sanitized
sanitized_rooms = []
list_rooms = [ "room1", "room2", "room3", "room4", "room5", "room6" ,"room7", "room8" ]


#function to comut the center of the room based on its corners
def compute_center_for_room(room):
    corners = room.values()  # Access values directly
    corners = [corner for corner in corners if corner is not None]  # Filter out None values

    if corners:
        # Extract x and y coordinates of all corners
        x_coords = [corner['x'] for corner in corners]
        y_coords = [corner['y'] for corner in corners]

        # Compute the center as the average of x and y coordinates
        center_x = np.mean(x_coords)
        center_y = np.mean(y_coords)

        # Assuming z, qx, qy, qz, qw remain the same for the center
        center = {'x': center_x, 'y': center_y, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}

        # Update the 'center' value in the room dictionary
        room['center'] = center
        return center



#functin to check in which room the robot is
def is_robot_inside_room(robot_pose, room_corners):
    x = robot_pose['x']
    y = robot_pose['y']

    corner1 = room_corners[0] # Top-left corner
    corner3 = room_corners[2] # Bottom-Right corner
 
    if corner1['x'] < x < corner3['x'] and corner3['y'] < y < corner1['y']:
        return True
    else:
        return False






##########################################################
#seting up the navigation strategy based on the static map
# and AMCL
##########################################################
class BasicNavigator(Node):
    def __init__(self):
        super().__init__('basic_navigator')
        self.initial_pose = Pose()
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        self.latest_odom_pose = None  # Variable to store the latest odometry pose
        self.path_length = None
        

        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5)

        self.initial_pose_received = False
        self.nav_through_poses_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.model_pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amclPoseCallback, amcl_pose_qos)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

       
        self.odom_pose_sub = self.create_subscription(Odometry, 'odom', self._odomPoseCallback, QoSProfile(depth=1))
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sanitization_status = self.create_publisher(String, 'san_status', 10)


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
    
    def computePathLength(self):
        return self.path_length

    def waitUntilNav2Active(self):
        self._waitForNodeToActivate('amcl')
        self._waitForInitialPose()
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

    def _setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        return


        
    def _odomPoseCallback(self, msg):
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
    
    
##########
#Main code
##########
def main(argv=sys.argv[1:]):

    rclpy.init()
    navigator = BasicNavigator()



    # function for creae a navigation goal
    def create_pose(x, y, z, qx, qy, qz, qw):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()

        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z
        goal_pose.pose.orientation.x = qx
        goal_pose.pose.orientation.y = qy
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw
        return goal_pose
    
    # Function to find the closest corner in a room 
    def find_closest_corner(robot_pose, room_corners):
        distances = [np.sqrt((robot_pose['x'] - corner['x'])**2 + (robot_pose['y'] - corner['y'])**2) for corner in room_corners]
        closest_corner_index = np.argmin(distances)
        return closest_corner_index



    # Function to generate poses based on the closest corner
    def generate_poses(closest_corner_index, room_corners, room_center):
        corner_sequence = room_corners[closest_corner_index:] + room_corners[:closest_corner_index+1]  # Clockwise order
        poses = [create_pose(corner['x'], corner['y'], corner['z'], corner['qx'], corner['qy'], corner['qz'], corner['qw']) for corner in corner_sequence]
        poses.append(create_pose(room_center['x'], room_center['y'], room_center['z'], room_center['qx'], room_center['qy'], room_center['qz'], room_center['qw']))
        return poses
        #function to santitaze the selected room
    """
    corner1:= top-left
    corner2:= top-right
    corner3:= bottom-right
    corner4:= bottom-left
    based on the room search for the closest corner and the robot performs a clock-wise movement 
    along the corner. for example, if room1 was selected and the robot is close to corner 1 it goes 
    theouge the corners the poses 1,2,3,4 then to the center.if room1 selected and it is close to 
    corner2 it does the movement 2,3,4,1 then to the center.
    """
    def sanitize_room(room_name, room_data, navigator):
        for room_name, room_data in room_reference_table.items():
            robot_pose = {'x': navigator.latest_odom_pose.position.x, 'y':navigator.latest_odom_pose.position.y, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}
            room_corners = [
                room_data['corner1'],
                room_data['corner2'],
                room_data['corner3'],
                room_data['corner4']
            ]
            if is_robot_inside_room(robot_pose, room_corners):
                print(f"The robot is inside {room_name}.")
                sanitized_rooms.append(room_name)
                #step2.1 Find the closest corner
                closest_corner_index = find_closest_corner(robot_pose, room_corners)

                #step2.2 Generate poses based on the closest corner
                room_center = room_data['center']
                poses_list = generate_poses(closest_corner_index, room_corners, room_center)

        # Send goals and wait until the robot reaches them
        for goal_pose in poses_list:
            navigator.goToPose(goal_pose)

            print('Navigator go to pose')
            i = 0
            while not navigator.isNavComplete():
                rclpy.spin_once(navigator, timeout_sec=0.1)  # Ensure the node is still spinning
                # Implement code for your application, check feedback, etc.

            print('Goal concluded')
            result = navigator.getResult()
            if result == GoalStatus.STATUS_SUCCEEDED:
                print('Goal succeeded!')
            elif result == GoalStatus.STATUS_CANCELED:
                print('Goal was canceled!')
            elif result == GoalStatus.STATUS_ABORTED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status.')
                # Retrieve feedback to get the executed path
            # Retrieve feedback to get the executed path
            feedback = navigator.getFeedback()
            if feedback:
                distance_remaining = feedback.distance_remaining
                print(f"Distance remaining in the executed path: {distance_remaining}")
    


    ######################################
    #Step0: send the intial Pose for amcl
    ######################################
    while not navigator.initial_pose_received:
            navigator.get_logger().info(f"wating for Initial Pose")
            rclpy.spin_once(navigator, timeout_sec=1.0)

    initial_pose = Pose()
    initial_pose.position.x =  navigator.latest_odom_pose.position.x
    initial_pose.position.y = navigator.latest_odom_pose.position.y
    initial_pose.orientation.z = navigator.latest_odom_pose.position.z
    #Orientation Values
    initial_pose.orientation.x = navigator.latest_odom_pose.orientation.x
    initial_pose.orientation.y = navigator.latest_odom_pose.orientation.y
    initial_pose.orientation.z = navigator.latest_odom_pose.orientation.z
    initial_pose.orientation.w = navigator.latest_odom_pose.orientation.w
    print('Sending initial pose...')
    navigator.setInitialPose(initial_pose)


    input('Pose ok, enter to continue')
    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()
    input('Navigation2 ok, enter to continue')

  
    ############################################################
    #setp1: define the room centers based on the reference table
    ############################################################
   # Iterate over rooms in the room_reference_table and compute centers
    for room_name, room_data in room_reference_table.items():
        compute_center_for_room(room_data)

    """ now we have to proceed to find the closest ro0m to sinitze 
    the robot must find the closest room based on the length of 
    the path from the center of the room that the robot in to the 
    center of the room that we will go to,after identifing the closest
    room we porceed with the same procedure which is define the 
    closest corener then do the clock wise movement"""
    
    ################################################################
    #step2: Start Sanitizization from the room that the robot inside
    ################################################################
    while len(sanitized_rooms) < 8:
        ####################
        #step1.1: snitize room
        ####################

          
        sanitize_room(room_name, room_data, navigator)
        print("the list of sanitized_rooms:")
        print(sanitized_rooms)

        #####################################################
        #step1.2: search for the closest room to be sanitized
        #####################################################
        closest_room_name = None
        min_path_length = float('inf')

        for room_name, room_data in room_reference_table.items():
            if room_name in sanitized_rooms:
                continue
            center = room_data['center']
            goal_pose = create_pose(center['x'], center['y'], center['z'], center['qx'], center['qy'], center['qz'], center['qw']) 
            navigator.goToPose(goal_pose)
            i = 0
            while not navigator.isNavComplete():
                rclpy.spin_once(navigator, timeout_sec=0.1)  # Ensure the node is still spinning
                time.sleep(0.1)
                feedback = navigator.getFeedback()
                path_length = feedback.distance_remaining
                navigator.cancelNav()
                print("path_length")
                print(path_length)

            print('Goal concluded')
            result = navigator.getResult()
            if result == GoalStatus.STATUS_SUCCEEDED:
                print('Goal succeeded!')
            elif result == GoalStatus.STATUS_CANCELED:
                print('searching for new room!')
            elif result == GoalStatus.STATUS_ABORTED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status.')
                # Retrieve feedback to get the executed path
            # Retrieve feedback to get the executed path
            feedback = navigator.getFeedback()
            if feedback:
                distance_remaining = feedback.distance_remaining
                print(f"Distance remaining in the executed path: {distance_remaining}")
        
            if path_length < min_path_length:
                min_path_length = path_length
                closest_room_name = room_name

        ##############################################
        #step1.3: go to the center of the closest room
        ##############################################
        print(f"The closest room is {closest_room_name} with a path length of {min_path_length}.")
        closest_room_data = room_reference_table[closest_room_name]
        center = closest_room_data["center"]
        goal_pose = create_pose(center['x'], center['y'], center['z'], center['qx'], center['qy'], center['qz'], center['qw'])

        navigator.goToPose(goal_pose)
        print('Navigator go to pose')
        i = 0
        while not navigator.isNavComplete():
            rclpy.spin_once(navigator, timeout_sec=0.1)  # Ensure the node is still spinning
            # Implement code for your application, check feedback, etc.

        print('Goal concluded')
        result = navigator.getResult()
        if result == GoalStatus.STATUS_SUCCEEDED:
            print('Goal succeeded!')
        elif result == GoalStatus.STATUS_CANCELED:
            print('Goal was canceled!')
        elif result == GoalStatus.STATUS_ABORTED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status.')
            # Retrieve feedback to get the executed path
        # Retrieve feedback to get the executed path
        feedback = navigator.getFeedback()
        if feedback:
            distance_remaining = feedback.distance_remaining
            print(f"Distance remaining in the executed path: {distance_remaining}")
    print("[INFO] SANITIZATION COMPLETED")
    status_msg = String()
    status_msg.data = "SANITIZATION COMPLETED"
    navigator.sanitization_status.publish(status_msg)
    # Repeat the process until all rooms are sanitized
if __name__ == '__main__':
    main()

