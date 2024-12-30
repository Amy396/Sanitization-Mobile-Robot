import itertools       #Provides the cycle function to loop through goals endlessly
import random
import yaml
import sys             #Allows program exit with sys.exit()
import rclpy
from rclpy.action import ActionClient         #Used to communicate with the Navigation2 stack for sending goals and paths.
from rclpy.node import Node
from geometry_msgs.msg import Point, Quaternion    #Message types for positions and orientations in 3D space
from nav2_msgs.action import NavigateToPose, FollowPath   #action type for sending pose and path
from std_msgs.msg import Bool

class Navigator(Node):
    """
    Node to send goals to the navigation2 stack based on a predefined route.

    Routes can be loaded from a YAML file with the following format:

        mode: inorder
        poses:
          - pose:
              position:
                x: -5.41667556763
                y: -3.14395284653
                z: 0.0
              orientation:
                x: 0.0
                y: 0.0
                z: 0.785181432231
                w: 0.619265789851

    Modes:
      - "inorder": Goals are visited sequentially.
      - "random": Goals are visited in random order.
    """

    route_modes = {
        'inorder': lambda goals: itertools.cycle(goals), #Cycles through goals sequentially.
        'random': lambda goals: (random.shuffle(goals), itertools.cycle(goals))[1],  ##Randomly shuffles the goals, then cycles through them endlessly.
    }

    def __init__(self):
        super().__init__('navigator')

        self.route = []
        self.current_goal = NavigateToPose.Goal()
        self.localization_complete = False

        # Subscribers
        self.create_subscription(Bool, 'localization_complete', self.bool_callback, 10)

        # Action clients
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.client.wait_for_server()

        self.follow_path_client = ActionClient(self, FollowPath, 'follow_path')
        self.follow_path_client.wait_for_server()

        # Load route from file
        route_file_path = r"/home/amy/burger_ws1/src/localization_sanitazation/localization_sanitazation/navgoals.yaml"
        with open(route_file_path, 'r') as f:
            route_yaml = yaml.safe_load(f)

        self.route_mode = route_yaml.get('mode', 'inorder')
        if self.route_mode not in Navigator.route_modes:
            self.get_logger().error(f"Unknown route mode '{self.route_mode}'. Exiting Navigator.")
            sys.exit(1)

        poses = route_yaml.get('poses', [])
        if not poses:
            self.get_logger().info("Navigator initialized with no goals.")
            sys.exit(1)

        self.goals = Navigator.route_modes[self.route_mode](poses)
        self.length = len(poses)
        self.number_of_goals = 0
        self.get_logger().info(f"Navigator initialized with {self.length} goals in {self.route_mode} mode.")

    def to_move_goal(self, pose):
        """Convert a pose dictionary to a NavigateToPose goal."""
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position = Point(**pose['pose']['position'])
        goal.pose.pose.orientation = Quaternion(**pose['pose']['orientation'])
        return goal

    def goal_response_callback(self, future):
        """Handle response when a goal is sent."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal rejected.')
            return
        self.get_logger().info('Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def route_forever(self):
        """Send the next goal to the navigation stack."""
        self.get_logger().info(f"Route mode is '{self.route_mode}', getting next goal.")
        try:
            current_goal = self.to_move_goal(next(self.goals))
        except StopIteration:
            self.get_logger().info("All goals visited. Stopping Navigator.")
            return

        self.number_of_goals += 1
        self.get_logger().info(f"Sending target goal: ({current_goal.pose.pose.position.x}, {current_goal.pose.pose.position.y}).")
        self._send_goal_future = self.client.send_goal_async(
            current_goal,
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def get_result_callback(self, future):
        """Handle result after goal is completed."""
        result = future.result().result
        self.get_logger().info(f'Result: {result.result}')
        if self.number_of_goals < self.length:
            self.route_forever()
        else:
            self.get_logger().info("No more goals. Stopping Navigator.")

    def feedback_callback(self, feedback_msg):
        """Handle feedback during navigation (not used)."""
        pass

    def bool_callback(self, msg):
        """Check if localization is complete."""
        self.localization_complete = msg.data


def main():
    rclpy.init()
    try:
        navigator = Navigator()
        navigator.get_logger().info("Waiting for localization to complete.")
        while not navigator.localization_complete:
            rclpy.spin_once(navigator)

        navigator.get_logger().info("Localization complete. Starting Navigator.")
        navigator.route_forever()

        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    except BaseException as e:
        print(f'Exception in navigator: {e}', file=sys.stderr)
        raise
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
