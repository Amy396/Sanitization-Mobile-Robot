import time, sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from nav_msgs.msg import OccupancyGrid
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
import numpy as np
import numpy as np
import math
from sensor_msgs.msg import Image
import yaml

class Sanitization(Node):
    def __init__(self):
        super().__init__('sanitization')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.listener_callback,
            10
            )
        self.map_data = None
        self.start_energy = False
        self.k=0
        self.occupancy_grid = None
        self.robot_pose = None
        self.initial_pose = Pose()
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        self.first_iter = True
        self.energy_grid = None
        self.sanitize = None
        self.visualize = None
        qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE)
        self.image_pub = self.create_publisher(Image, '/Image', 10)
        self.nav_to_pose_client = ActionClient (self, NavigateToPose, 'navigate_to_pose')
        self.nav_through_poses_client = ActionClient (self, NavigateThroughPoses, 'navigate_through_poses')
        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        self.model_pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amclPoseCallback, amcl_pose_qos)
        self.energy_map_publisher = self.create_publisher(OccupancyGrid, '/energy_map', qos)
     
    def world_to_map(self, world_x, world_y):
        ''' 
        Convert world coordinates to map coordinates
        - Input: world_x, world_y
        - Output: map_x, map_y 
        '''


        origin = self.occupancy_grid.info.origin.position
        resolution = self.occupancy_grid.info.resolution

        map_x = (world_x - origin.x) / resolution
        map_y = (world_y - origin.y) / resolution

        return int(map_x), int(map_y)
    
    def map_to_world(self, map_x, map_y):
        '''
        Convert map coordinates to world coordinates
        - Input: map_x, map_y
        - Output: world_x, world_y
        '''
        origin = self.occupancy_grid.info.origin.position
        resolution = self.occupancy_grid.info.resolution

        world_x = map_x * resolution + origin.x
        world_y = map_y * resolution + origin.y

        return world_x, world_y
        
    def listener_callback(self, msg):
        '''
        Callback function for the occupancy grid subscriber
        '''
        self.occupancy_grid = msg 
        self.map_data = np.array(self.occupancy_grid.data)
        #reshape mapdata to 2d array to match as occupancy grid
        self.map_data = np.reshape(self.map_data, (self.occupancy_grid.info.height, self.occupancy_grid.info.width))

        self.map_data = np.where((self.map_data >= 0) & (self.map_data <= 97), 0, self.map_data)
        self.map_data = np.where(self.map_data > 97, 100, self.map_data)

        if self.k == 0:  # Initialize sanitize and related grids and update them based on message we received
            self.energy_grid = np.zeros((self.occupancy_grid.info.height, self.occupancy_grid.info.width), dtype=np.float128)
            self.sanitize = np.zeros((self.occupancy_grid.info.height, self.occupancy_grid.info.width), dtype=np.float32)
            self.visualize = np.zeros((self.occupancy_grid.info.height, self.occupancy_grid.info.width), dtype=np.uint8)
            self.k = 1

        self.energy_map = self.update()

    
    def _amclPoseCallback(self, msg):
        '''
        Callback function for the AMCL pose subscriber to update robot pose
        '''
        self.initial_pose = msg.pose.pose
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_pose = msg.pose.pose
        # print("Robot position: ", self.robot_x, self.robot_y)
    
    def goThroughPoses(self, poses):
        '''
        Sends a `NavigateThroughPoses` action request and waits for completion
        '''
       
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses

        self.info('Navigating with ' + str(len(poses)) + ' goals.' + '...')
        send_goal_future = self.nav_through_poses_client.send_goal_async(goal_msg,
                                                                        self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal with ' + str(len(poses)) + ' poses was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True
    
    def goToPose(self, pose):
        '''
        Sends a `NavigateToPose` action request and waits for completion
        '''
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                        str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                    self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                        str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True
    
    def _feedbackCallback(self, msg):
        '''
        Callback function for the feedback subscriber from navigation action
        '''
        # local copy of the feedback callback for future use
        self.feedback = msg.feedback
        return

    def cancelNav(self):
        self.info('Canceling current goal.')
        if self.result_future:    ## if a goal is on process
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isNavComplete(self):
        '''
        Checks if the navigation action is complete
        '''
        if not self.result_future:
            # the goal is not on process so means task was cancelled or completed
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

    def getFeedback(self):      #get feedback of navigation action
        return self.feedback

    def getResult(self):       #get status of navigation action
        return self.status

    def waitUntilNav2Active(self):
        self._waitForNodeToActivate('amcl')
        self._waitForInitialPose()
        self._waitForNodeToActivate('bt_navigator')
        self.info('Nav2 is ready for use!')
        return

    def _waitForNodeToActivate(self, node_name):
        self.debug('Waiting for ' + node_name + ' to become active..')
        node_service = node_name + '/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(node_service + ' service not available, waiting...')

        req = GetState.Request()    #Continuously sends requests to the node’s /get_state service until it reports an "active" state.
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
    
    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return
    
    def calculate_energy(self, dx, dy):
        '''
        Calculate the energy for a cell based on the distance from the robot
        - Input: dx, dy (distance from the robot in x and y directions)
        - Output: energy for the cell
        '''
        distance_in_map_units = np.sqrt(dx**2 + dy**2)

        # Convert the distance to world units
        distance_in_world_units = distance_in_map_units * self.occupancy_grid.info.resolution
        pixel_2 = self.occupancy_grid.info.resolution**2  #area of each cell
        pi = 100*1e-6  #power unit = watt.square meter
        process_speed_factor = 1e4 ### scale energy For simulation purposes only because value is small

        if distance_in_world_units > (0.1*self.occupancy_grid.info.resolution):
            energy = process_speed_factor*(pi * pixel_2)/(dx**2 + dy**2)
            # print("Energy: ", energy)
        else:
            energy = 0
        return energy

    def update(self):
        '''
        Update the energy grid based on the occupancy grid subscriber(obstacles and free space) and the robot position via AMCL subscriber.
        '''
        
        if self.occupancy_grid is not None and self.start_energy == True:

            if self.k==0:
                #tracks energy values.
                self.energy_grid = np.zeros((self.occupancy_grid.info.height, self.occupancy_grid.info.width), dtype=np.float128)
                #marks sanitized cells
                self.sanitize = np.zeros((self.occupancy_grid.info.height, self.occupancy_grid.info.width), dtype=np.float32)
                #stores color-coded energy levels for visualization
                self.visualize = np.zeros((self.occupancy_grid.info.height, self.occupancy_grid.info.width), dtype=np.uint8)
                                
                self.k=1
            
            position = self.robot_pose
            #Retrieves the robot's world position and converts it to map coordinates because we want to show them on map
            if position is not None:
                robot_world_x, robot_world_y = position.position.x, position.position.y
                robot_x, robot_y = self.world_to_map(robot_world_x, robot_world_y)
                
                num_directions = 360    #The number of directions to simulate energy propagation 
                angle_increment = 360 / num_directions    #The step size for iterating over angles
                energy_threshold =  1e-3    ####Minimum energy required to sanitize a cell

                for angle in range(0, 360, int(angle_increment)):
                    angle_rad = np.radians(angle)
                    ii, jj = 0, 0
                    
                    while True:
                        ii += np.cos(angle_rad)
                        jj += np.sin(angle_rad)

                        # Check if the cell is within the map boundaries
                        if 0 <= robot_x + int(ii) < self.map_data.shape[1] and 0 <= robot_y + int(jj) < self.map_data.shape[0]:
                            # Check if there is an obstacle in the current direction
                            if self.map_data[robot_y + int(jj), robot_x + int(ii)] == 100:
                                self.sanitize[robot_y + int(jj), robot_x + int(ii)] = 1
                                break  # Stop spreading energy in this direction if an obstacle is encountered
                            else:
                                # Calculate energy for the cell
                                cell_energy = self.calculate_energy(ii, jj)
                                self.energy_grid[robot_y + int(jj), robot_x + int(ii)] += cell_energy

                            if self.energy_grid[robot_y + int(jj), robot_x + int(ii)] < (1e-11):
                                self.visualize[robot_y + int(jj), robot_x + int(ii)] = 0
                                self.sanitize[robot_y + int(jj), robot_x + int(ii)] = 0
                            elif self.energy_grid[robot_y + int(jj), robot_x + int(ii)] >= (1e-11) and self.energy_grid[robot_y + int(jj), robot_x + int(ii)] < (1e-9):
                                self.visualize[robot_y + int(jj), robot_x + int(ii)] = 0
                                self.sanitize[robot_y + int(jj), robot_x + int(ii)] = 0
                            elif self.energy_grid[robot_y + int(jj), robot_x + int(ii)] >= (1e-9) and self.energy_grid[robot_y + int(jj), robot_x + int(ii)] < (1e-7):
                                self.visualize[robot_y + int(jj), robot_x + int(ii)] = 10
                                self.sanitize[robot_y + int(jj), robot_x + int(ii)] = 0
                            elif self.energy_grid[robot_y + int(jj), robot_x + int(ii)] >= (1e-7) and self.energy_grid[robot_y + int(jj), robot_x + int(ii)] < (1e-5):
                                self.visualize[robot_y + int(jj), robot_x + int(ii)] = 10
                                self.sanitize[robot_y + int(jj), robot_x + int(ii)] = 0
                            elif self.energy_grid[robot_y + int(jj), robot_x + int(ii)] >= (1e-5) and self.energy_grid[robot_y + int(jj), robot_x + int(ii)] < (1e-3):
                                self.visualize[robot_y + int(jj), robot_x + int(ii)] = 50
                                self.sanitize[robot_y + int(jj), robot_x + int(ii)] = 0
                            elif self.energy_grid[robot_y + int(jj), robot_x + int(ii)] >= (energy_threshold):
                                self.visualize[robot_y + int(jj), robot_x + int(ii)] = -100    ### 100 means occupied -100 means sanitized in visualization
                                self.sanitize[robot_y + int(jj), robot_x + int(ii)] = 1         ###1 means sanitized in sanitized grid
                        else:
                            break
                
                # match visualization to occupancy grid and then Publish the visualization on the energy_map topic
                visualization_msg = OccupancyGrid()
                visualization_msg.header.stamp = self.get_clock().now().to_msg()
                visualization_msg.header.frame_id = 'map'
                visualization_msg.info.width = self.occupancy_grid.info.width
                visualization_msg.info.height = self.occupancy_grid.info.height
                visualization_msg.info.resolution = self.occupancy_grid.info.resolution
                visualization_msg.info.origin = self.occupancy_grid.info.origin
                visualization_msg.data = np.clip(self.visualize.flatten().astype(int), -128, 127).tolist()
                self.energy_map_publisher.publish(visualization_msg)

            else:
                self.get_logger().info('Robot position not available yet')
                            
        else:
            # self.get_logger().info('Occupancy grid not received yet')
            pass
       

        return self.energy_grid
        
def create_pose(x, y):
    '''
    Create a PoseStamped message with the given x and y coordinates
    '''
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.w = 1.0
    return pose

def calculate_distance(point1, point2):
    '''
    Calculate the distance between two points
    '''
    point2_x, point2_y = map(float, point2)
    dx = point2_x - point1.position.x
    dy = point2_y - point1.position.y
    return math.sqrt(dx**2 + dy**2)

    
def give_corners(room):
    '''
    Return the corners of the room
    - Input: room
    - Output: room_corners
    '''
    
    room_coordinates = {
        'dining_room': [5.0, 7.2, -5.2, 0.0],  # x_start, x_end, y_start, y_end
        'kitchen': [2.2, 7.2, 0.0, 5.0],
        'toilet': [0.0, 2.2, 1.0, 5.0],
        'living_room': [-5.0, 0.0, 0.0, 5.0],
        'bedroom1': [-7.2, -5.0, 1.0, 5.0],
        'bedroom2': [-7.2, -5.0, -4.0, 1.0],
    }


    wall_offset = 0.3  # for Avoiding collisions with walls
    coordinates = room_coordinates[room]
    return [
        (coordinates[0] + wall_offset, coordinates[2] + wall_offset),  # Bottom-left
        (coordinates[0] + wall_offset, coordinates[3] - wall_offset),  # Top-left
        (coordinates[1] - wall_offset, coordinates[3] - wall_offset),  # Top-right
        (coordinates[1] - wall_offset, coordinates[2] + wall_offset),  # Bottom-right
        (coordinates[0] + wall_offset, coordinates[2] + wall_offset)   # Closing loop
    ]


def give_center(room):
    '''
    Return the center of the room
    - Input: room
    - Output: room_center_x, room_center_y
    '''

    room_coordinates = {
        'dining_room': [5.0, 7.2, -5.2, 0.0],  # x_start, x_end, y_start, y_end
        'kitchen': [2.2, 7.2, 0.0, 5.0],
        'toilet': [0.0, 2.2, 1.0, 5.0],
        'living_room': [-5.0, 0.0, 0.0, 5.0],
        'bedroom1': [-7.2, -5.0, 1.0, 5.0],
        'bedroom2': [-7.2, -5.0, -4.0, 1.0],
    }


    coordinates = room_coordinates[room]
    x_start, x_end, y_start, y_end = coordinates
    room_center_x = (x_start + x_end) / 2
    room_center_y = (y_start + y_end) / 2
    return room_center_x, room_center_y


def check_room(robot_x, robot_y):
    '''
    Check which room the robot is in based on its coordinates
    Input: robot_x, robot_y, this is the position of the robot
    Output: room, the room the robot is in
    '''
    room_coordinates = {
        'dining_room': [5.0, 7.2, -5.2, 0.0],  # x_start, x_end, y_start, y_end
        'kitchen': [2.2, 7.2, 0.0, 5.0],
        'toilet': [0.0, 2.2, 1.0, 5.0],
        'living_room': [-5.0, 0.0, 0.0, 5.0],
        'bedroom1': [-7.2, -5.0, 1.0, 5.0],
        'bedroom2': [-7.2, -5.0, -4.0, 1.0],  
    }
    for room, coordinates in room_coordinates.items():
        x_start, x_end, y_start, y_end = coordinates
        if x_start <= robot_x <= x_end and y_start <= robot_y <= y_end:
            return room
    return None

def is_point_in_room(point, room_corners):
    '''
    Check if a point is inside a room
    - Input: point, room_corners
    - Output: inside, True if the point is inside the room, False otherwise
    '''
    x, y = point
    inside = False
    for i in range(len(room_corners)):
        j = (i + 1) % len(room_corners)
        if (room_corners[i][1] > y) != (room_corners[j][1] > y) and \
                x < (room_corners[j][0] - room_corners[i][0]) * (y - room_corners[i][1]) / \
                (room_corners[j][1] - room_corners[i][1]) + room_corners[i][0]:
            inside = not inside
    return inside

def main(args=None):
    rclpy.init(args=args)
    sanitization = Sanitization()
    while sanitization.map_data is None:
        rclpy.spin_once(sanitization)
        
    

    # rooms = [
    #     'dining_room',
    #     'kitchen',
    #     'toilet',
    #     'living_room',
    #     'bedroom1',
    #     'bedroom2',
    # ]
    
    # Read the rooms to be sanitized from the yaml file
    with open('/home/amy/burger_ws1/src/localization_sanitazation/localization_sanitazation/rooms.yaml', 'r') as file:
        rooms_data = yaml.safe_load(file)
    rooms = rooms_data['rooms']  # Extract the list of room names

    original_rooms = np.copy(rooms)

    print ("List of rooms to be sanitized: ", rooms)
    while sanitization.robot_pose is None:
        rclpy.spin_once(sanitization)
        print("Waiting for robot pose...")
    
    robot_x = sanitization.robot_pose.position.x
    robot_y = sanitization.robot_pose.position.y

    room = check_room(robot_x, robot_y)
    current_room = room
    print("Robot spawned in", current_room)

    if room in rooms:
        current_room = room
    else:
        nearest_room = None
        min_distance = float('inf')

        for room in rooms:
            room_center_x, room_center_y = give_center(room)
            distance = calculate_distance((sanitization.robot_pose), (room_center_x, room_center_y))
            if distance < min_distance:
                min_distance = distance
                nearest_room = room

        if nearest_room is not None:
            print("Going to nearest room:", nearest_room)
            room_center_x, room_center_y = give_center(nearest_room)
            goal_pose = create_pose(room_center_x, room_center_y)
            sanitization.goToPose(goal_pose)
            while not sanitization.isNavComplete():
                rclpy.spin_once(sanitization)

            while sanitization.status != GoalStatus.STATUS_SUCCEEDED:
                sanitization.goToPose(goal_pose)
                while not sanitization.isNavComplete():
                    rclpy.spin_once(sanitization)
            
            robot_pose = sanitization.robot_pose
            robot_x = sanitization.robot_x
            robot_y = sanitization.robot_y
    
    current_room = check_room(robot_x, robot_y)
    sanitization.start_energy = True
    if current_room is not None:
        print("Robot is in", current_room)
    else:
        print("Robot is not in any room")

    room_center_x,room_center_y = give_center(current_room)
    room_corners = give_corners(current_room)

    goal_pose = create_pose(room_center_x, room_center_y)
    sanitization.goToPose(goal_pose)
    print("I am going to the center of the room")
    while not sanitization.isNavComplete():
        rclpy.spin_once(sanitization)

    robot_pose = sanitization.robot_pose
    robot_x = robot_pose.position.x
    robot_y = robot_pose.position.y
    robot_pose = sanitization.robot_pose
            
    print("I am going to the corners of the room")
    
    house_sanitized = False 

    completed_rooms = []  
    
    ## Main Loop to start Sanitizing the rooms

    while house_sanitized == False: 
        room_complete = False
        
        for corner in room_corners:
            goal_pose = create_pose(corner[0], corner[1])
            sanitization.goToPose(goal_pose)
            while not sanitization.isNavComplete():                
                rclpy.spin_once(sanitization)

        while room_complete == False:
            iter = 0
            robot_pose = sanitization.robot_pose
            robot_x = robot_pose.position.x
            robot_y = robot_pose.position.y

            unsanitized_points = []
            nearest_coordinate = None
            
            if sanitization.sanitize is None:
                print("Sanitize grid not initialized. Waiting for occupancy grid...")
                while sanitization.sanitize is None:
                    rclpy.spin_once(sanitization)

            
            # print(" first: ", unsanitized_points)
            for i in range(sanitization.sanitize.shape[0]):
                for j in range(sanitization.sanitize.shape[1]):
                    if sanitization.sanitize[i, j] != 1:
                        world_x, world_y = sanitization.map_to_world(i, j)
                        world_x = round(world_x,3)
                        world_y = round(world_y,3)
                        point = (world_x, world_y)
                        if is_point_in_room(point, room_corners):
                            unsanitized_points.append(point)
                            

            # list to Remove repeated points
            unsanitized_points = list(set(unsanitized_points))

            # filter to Remove points that are too close to each other
            filtered_coordinates = []
            filtering_threshold = 0.4
            for i in range(len(unsanitized_points)):
                is_close = False
                for j in range(i+1, len(unsanitized_points)):
                    distance = math.sqrt((unsanitized_points[i][0] - unsanitized_points[j][0])**2 + (unsanitized_points[i][1] - unsanitized_points[j][1])**2)
                    if distance < filtering_threshold:
                        is_close = True
                        break
                if not is_close:
                    filtered_coordinates.append(unsanitized_points[i])
        
            unsanitized_points = filtered_coordinates
            unsanitized_points.sort(key=lambda coord: math.sqrt((coord[0] - robot_x)**2 + (coord[1] - robot_y)**2))

            print("There were ", len(unsanitized_points),"unsanitized points found: ", unsanitized_points)

            # Use the nearest_coordinate for further processing
            while unsanitized_points:
                nearest_coordinate = min(unsanitized_points, key=lambda coord: math.sqrt((coord[0] - robot_x)**2 + (coord[1] - robot_y)**2))
                map_nearest_coordinate = sanitization.world_to_map(nearest_coordinate[0], nearest_coordinate[1])
                if sanitization.sanitize[map_nearest_coordinate[0], map_nearest_coordinate[1]] == 1:
                    unsanitized_points.remove(nearest_coordinate)
                    continue
                goal_pose = create_pose(nearest_coordinate[0], nearest_coordinate[1])
                sanitization.goToPose(goal_pose)

                while not sanitization.isNavComplete():
                    rclpy.spin_once(sanitization)
                
                i, j = sanitization.world_to_map(nearest_coordinate[0], nearest_coordinate[1])

                # Remove visited point from unsanitized_points
                unsanitized_points.remove(nearest_coordinate)                             

                # Update robot position
                
            room_complete = True
            iter +=1
                # Check if all coordinates in the room have been set to 1
            if iter == 0:
                for i in range(sanitization.sanitize.shape[0]):
                    for j in range(sanitization.sanitize.shape[1]):
                        world_x, world_y = sanitization.map_to_world(i, j)
                        world_x = round(world_x,3)
                        world_y = round(world_y,3)
                        point = (world_x, world_y)
                        if is_point_in_room(point, room_corners):
                            if sanitization.sanitize[i, j] == 0:
                                room_complete = False
                                break
                        


                            

        robot_pose = sanitization.robot_pose
        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y  
        print(current_room," has been sanitized")         

        current_room_center_x, current_room_center_y = give_center(current_room)
        
        completed_rooms.append(current_room)  # add curent room to completed and deleted from curent room

        rooms.remove(current_room)

        print("Completed rooms: ", completed_rooms)
        print("Remaining rooms: ", rooms)
        

        if set(completed_rooms) == set(original_rooms):
            house_sanitized = True
            print("I have sanitized all the rooms that required sanitization. I am done!")
            break

        # Compute the center of the nearest room
        nearest_room_center = None

        distances = {}
        for room_name in rooms:
            center_x, center_y = give_center(room_name) 
            distance = math.sqrt((current_room_center_x - center_x)**2 + (current_room_center_y - center_y)**2)
            distances[room_name] = distance
        # print("Distances: ", distances)

        if distances:
            nearest_room = min(distances, key=distances.get)

        nearest_room_center = give_center(nearest_room)

        if nearest_room_center is not None:
            center_x, center_y = nearest_room_center
            current_room = nearest_room
            room_corners = give_corners(nearest_room)
            print("I am going to the center of the ",nearest_room)
            # print("Coordinates of the center of the nearest room: ", center_x, center_y)

        # Go to the center of the nearest room
        if nearest_room_center is not None:
            goal_pose = create_pose(nearest_room_center[0], nearest_room_center[1])
            sanitization.goToPose(goal_pose)
            print("Going to the center of the nearest room")
            while not sanitization.isNavComplete():
                rclpy.spin_once(sanitization)       

    try:
        rclpy.spin(sanitization)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        sanitization.destroy_node()
        rclpy.shutdown()

    sanitization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()