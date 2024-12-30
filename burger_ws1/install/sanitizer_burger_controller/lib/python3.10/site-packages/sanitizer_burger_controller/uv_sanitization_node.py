import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
from PIL import Image
import yaml
import os
import math
from std_msgs.msg import String


NUM_CELLS = 0 #Global variable for number of cells that can be sanitized
class UVSanitizationNode(Node):



    ###############
    #Initialization
    ###############
    def __init__(self):
        super().__init__('uv_sanitization_node')
         # Load and process the static map first to get necessary info
        self.uv_power = 100e-6   # UV power in W/m^2
        self.delta_t = 2.0       # Sample time in seconds
        self.energy_threshold = 10e-3  # Energy threshold in Joules (10 mJ)
        self.NUM_CELLS = 0 
        self.status_received = False

        # Initialize occupancy grid for UV light distribution
        self.occupancy_grid_pub = self.create_publisher(OccupancyGrid, 'uv_light_distribution', 10)
        self.static_map_pub = self.create_publisher(OccupancyGrid, 'san_status', 10)
        self.timer = self.create_timer(1.0,self.publish_static_map) # publish the map every 1 sec
        #subscribtion        
        self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.subscription = self.create_subscription(String,'san_status',self.status_callback,10)

        self.occupancy_grid = OccupancyGrid()
        self.load_static_map()
        self.initialize_occupancy_grid()


    ################################
    #Function for loading the static 
    ################################   
    def load_static_map(self):
        # Load the map metadata from the YAML file
        with open(os.path.expanduser('~/map_big_house.yaml'), 'r') as yaml_file:
            map_metadata = yaml.safe_load(yaml_file)

        # Explicitly convert resolution to float to ensure correct type
        self.map_resolution = float(map_metadata['resolution'])
        map_image = Image.open(os.path.expanduser('~/map_big_house.pgm'))
        map_array = np.array(map_image)

        # Store map properties
        self.map_origin = map_metadata['origin']
        self.grid_size_x, self.grid_size_y = map_array.shape[::-1]  # Width (x) and Height (y) swapped
        self.static_map = np.where(map_array <= 255 * map_metadata['occupied_thresh'], 100, 
                                   np.where(map_array >= 255 * map_metadata['free_thresh'], 0, -1))
        
        # Publish static map for visualization
        self.publish_static_map()

        # Count the number of cells that are either obstacles (100) or unknown (-1)
        num_obstacle_or_unknown_cells = np.sum(np.logical_or(self.static_map == 100, self.static_map == -1))
        self.NUM_CELLS = np.sum(np.logical_or.reduce((self.static_map == 100, self.static_map == -1, self.static_map == 0)))

        self.get_logger().info(f'Number of cells: {self.NUM_CELLS}')
        self.get_logger().info(f'Number of cells that are obstacles: {num_obstacle_or_unknown_cells}')
        self.get_logger().info(f'Number of cells that can be sanitized: {self.NUM_CELLS-num_obstacle_or_unknown_cells}')
         

        
    #publish static map on topic topic_
    def publish_static_map(self):
        # self.get_logger().info("Publishing static map...")
        static_grid = OccupancyGrid()
        static_grid.header.stamp = self.get_clock().now().to_msg()
        static_grid.header.frame_id = "map"
        static_grid.info.map_load_time = self.get_clock().now().to_msg()
        static_grid.info.resolution = self.map_resolution
        static_grid.info.width = self.grid_size_x
        static_grid.info.height = self.grid_size_y

        #initialize the Pose message for the origin
        origin_pose = Pose()
        origin_pose.position.x = self.map_origin[0]
        origin_pose.position.y = self.map_origin[1] +13.6
        origin_pose.position.z = 0.0
        origin_pose.orientation.x = 1.0
        origin_pose.orientation.y = 0.0
        origin_pose.orientation.z = 0.0
        origin_pose.orientation.w = 0.0
        static_grid.info.origin = origin_pose

        static_grid.data = self.static_map.ravel().tolist()
        self.static_map_pub.publish(static_grid)

    # Iitialize occupancy grid for sanitizing
    def initialize_occupancy_grid(self):
        self.occupancy_grid.header = Header(frame_id='map')
        self.occupancy_grid.info.resolution = self.map_resolution  
        self.occupancy_grid.info.width = self.grid_size_x
        self.occupancy_grid.info.height = self.grid_size_y
        origin_pose = Pose()
        origin_pose.position.x = self.map_origin[0]
        origin_pose.position.y = self.map_origin[1]
        origin_pose.position.z = 0.0
        origin_pose.orientation.x = 0.0
        origin_pose.orientation.y = 0.0
        origin_pose.orientation.z = 0.0
        origin_pose.orientation.w = 1.0
        self.occupancy_grid.info.origin = origin_pose
        self.occupancy_grid.data = [-1] * (self.grid_size_x * self.grid_size_y)  # Initialize as unknown
    
    #odometry callback
    def odom_callback(self, msg):
        px, py = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.update_energy_grid(px, py)
        self.update_occupancy_grid()
    # exploration satuts callback
    def status_callback(self,msg):
        self.status_msg = msg
        print(self.status_msg)
        self.status_received = True
    
    # #function for updatinf the energy level of the occupanct grid
    # def update_energy_grid(self, px, py):
    #     if not hasattr(self, 'energy_grid'):
    #         # self.get_logger().info("Intializing the energy_grid...")
    #         self.energy_grid = np.zeros((self.grid_size_y, self.grid_size_x))
            
    #     max_radius_cells = int(1.5 / self.map_resolution )  # For a 1.5-meter radius

    #     robot_cell_x = int((px - self.map_origin[0]) / self.map_resolution )
    #     robot_cell_y = int((py - self.map_origin[1]) / self.map_resolution )
     
    #     for dy in range(-max_radius_cells, max_radius_cells + 1):
    #         for dx in range(-max_radius_cells, max_radius_cells + 1):
    #             cell_x = robot_cell_x + dx
    #             cell_y = robot_cell_y + dy

    #             if 0 <= cell_x < self.grid_size_x and 0 <= cell_y < self.grid_size_y:
    #                 distance = math.sqrt(dx**2 + dy**2) * self.map_resolution

    #                 if distance <= 1.5 and self.is_visible(px, py, cell_x, cell_y):  # Within a 1.5-meter radius and visible
    #                     energy = self.uv_power * self.delta_t / (distance + 1)  # Simplified energy update
    #                     self.energy_grid[cell_y][cell_x] += energy



    def update_energy_grid(self, px, py):
        if not hasattr(self, 'energy_grid'):
            self.energy_grid = np.zeros((self.grid_size_y, self.grid_size_x))
        
        max_radius_cells = int(2.0 / self.map_resolution)  # For a 1.5-meter radius
        max_energy_cap = 0.04  # Example maximum energy cap

        robot_cell_x = int((px - self.map_origin[0]) / self.map_resolution)
        robot_cell_y = int((py - self.map_origin[1]) / self.map_resolution)

        for dy in range(-max_radius_cells, max_radius_cells + 1):
            for dx in range(-max_radius_cells, max_radius_cells + 1):
                cell_x = robot_cell_x + dx
                cell_y = robot_cell_y + dy

                if 0 <= cell_x < self.grid_size_x and 0 <= cell_y < self.grid_size_y:
                    distance = math.sqrt(dx**2 + dy**2) * self.map_resolution
                    if distance <= 2.0 and self.is_visible(px, py, cell_x, cell_y):  # Within a 1.5-meter radius and visible
                        additional_energy = self.uv_power * self.delta_t / (distance + 1)  # Simplified energy update
                        # Cap the energy at the maximum allowed value
                        self.energy_grid[cell_y][cell_x] = min(self.energy_grid[cell_y][cell_x] + additional_energy, max_energy_cap)



    # def update_energy_grid(self, px, py):
    #     # Initialize energy grid if not already done
    #     if not hasattr(self, 'energy_grid'):
    #         self.energy_grid = np.zeros((self.grid_size_y, self.grid_size_x))

    #     # Update energy based on robot position and visibility
    #     for y in range(self.grid_size_y):
    #         for x in range(self.grid_size_x):
    #             if self.is_visible(px, py, x, y):
    #                 distance = math.sqrt((self.map_origin[0] + x * self.map_resolution - px) ** 2 +
    #                                      (self.map_origin[1] + y * self.map_resolution - py) ** 2)
                    
                 
    #                 if distance <= 1.5:  # Update within 1.5 meters radius
    #                     self.energy_grid[y, x] += self.uv_power * self.delta_t / max(distance, 0.1)  # Avoid division by zero

    def is_visible(self, px, py, grid_x, grid_y):

        # Adjust py for translation
        adjusted_py = py + 13.6
        # Convert real-world coordinates to grid indices
        robot_x = int((px - self.map_origin[0]) / self.map_resolution)
        robot_y = int((adjusted_py - self.map_origin[1]-13.6) / self.map_resolution)
        robot_y = self.grid_size_y - 1 - robot_y
        grid_y = self.grid_size_y - 1 - grid_y

        # Bresenham's line algorithm for visibility check
        dx, dy = abs(grid_x - robot_x), abs(grid_y - robot_y)
        sx, sy = (1, -1)[robot_x > grid_x], (1, -1)[robot_y > grid_y]
        err, e2 = dx - dy, 0

        while True:
            if robot_x == grid_x and robot_y == grid_y: return True
            if self.static_map[robot_y, robot_x] == 100: return False  # Obstacle encountered
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                robot_x += sx
            if e2 < dx:
                err += dx
                robot_y += sy

    def update_occupancy_grid(self):
        # Convert energy grid to occupancy grid values
        max_energy = np.max(self.energy_grid)
        self.occupancy_grid.data = ((self.energy_grid / max_energy * 100).clip(0, 100) if max_energy > 0 else np.zeros_like(self.energy_grid)).ravel().astype(int).tolist()
        self.occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        self.occupancy_grid_pub.publish(self.occupancy_grid)

        # Count the number of cells with energy >= 10mJ
        
        if self.NUM_CELLS > 0: # and self.status_received == True:
            cells_above_threshold = np.count_nonzero(self.energy_grid >= self.energy_threshold )
            self.get_logger().info(f'number of cells with energy >= 10mJ: {cells_above_threshold}')
            area_per = 0
            print(self.NUM_CELLS)
            area_per = 100*cells_above_threshold / self.NUM_CELLS
            self.get_logger().info(f'The sanitized area : {area_per} %')
        



def main(args=None):
    
    rclpy.init(args=args)
    node = UVSanitizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
























