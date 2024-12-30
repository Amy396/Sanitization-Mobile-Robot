import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile    # to costum qualito of servise of AMCL SERVICE
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped  #Pose information with covariance for uncertainty.
from std_srvs.srv import Empty    #Service type for calling another service
from std_msgs.msg import Bool, Header


class Localization(Node):
    def __init__(self):
        super().__init__('localization')

        # Publisher to notify when localization is complete
        self.localization_complete_pub = self.create_publisher(Bool, 'localization_complete', 10)

        # AMCL pose subscriber with custom QoS settings
        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.amcl_pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.amcl_pose_callback, amcl_pose_qos)

        # Odometry subscriber to get the initial rough pose
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)

        # Publisher to set the initial pose of the robot
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

        # Service client to reinitialize global localization
        self.reinitialize_global_localization_client = self.create_client(Empty, '/reinitialize_global_localization')
        while not self.reinitialize_global_localization_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for reinitialize_global_localization service...')

        # Initial pose and covariance values
        self.initial_pose = None
        self.x_cov, self.y_cov, self.z_cov = float('inf'), float('inf'), float('inf')
        self.average_covariance = float('inf')
        self.localization_done = False

    def odometry_callback(self, msg):
        """Update initial pose using odometry data."""
        self.initial_pose = msg.pose.pose

    def amcl_pose_callback(self, msg):
        """Update pose and covariance values using AMCL data."""
        self.initial_pose = msg.pose.pose
        self.x_cov = msg.pose.covariance[0]  #covariance x wrt x
        self.y_cov = msg.pose.covariance[7]   #covariance y wrt y
        self.z_cov = msg.pose.covariance[35]   # covariance yaw wrt yaw
        self.average_covariance = (self.x_cov + self.y_cov) / 2


def main(args=None):
    rclpy.init(args=args)

    localization = Localization()

    # Wait until initial pose is received
    while localization.initial_pose is None:
        rclpy.spin_once(localization)

    # Publish initial pose
    initial_pose_msg = PoseWithCovarianceStamped()
    initial_pose_msg.header = Header(frame_id='map')
    initial_pose_msg.pose.pose = localization.initial_pose
    localization.initial_pose_pub.publish(initial_pose_msg)
    localization.get_logger().info('Initial pose set.')

    # Reinitialize global localization service request
    reinit_request = Empty.Request()
    localization.reinitialize_global_localization_client.call_async(reinit_request)

    # Covariance thresholds
    covariance_threshold_x = 1.0
    covariance_threshold_y = 1.5
    covariance_threshold_z = 0.5
    k = 0  # Iteration counter

    # Continuously check and refine localization
    while (localization.x_cov > covariance_threshold_x or
           localization.y_cov > covariance_threshold_y or
           localization.z_cov > covariance_threshold_z):

        rclpy.spin_once(localization)
        k += 1

        if k % 20000 == 0 and localization.average_covariance > 1.25 * ((covariance_threshold_x + covariance_threshold_y) / 2):
            localization.get_logger().info('Reinitializing localization...')
            localization.reinitialize_global_localization_client.call_async(reinit_request)
            localization.initial_pose_pub.publish(initial_pose_msg)

    # Publish localization completion
    localization.localization_done = True
    localization.localization_complete_pub.publish(Bool(data=True))
    localization.get_logger().info('Localization complete!')

    rclpy.spin(localization)
    localization.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()