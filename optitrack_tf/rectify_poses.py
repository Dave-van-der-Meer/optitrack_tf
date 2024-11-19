"""
This script subscribes to the pose topic and rectifies the pose,
by switching x, y and z to correspond to the ROS coordinate frame.
"""
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class PoseRectifyer(Node):
    """
    This class subscribes to the pose topic and rectifies the pose,
    by switching x, y and z to correspond to the ROS coordinate frame.
    """
    def __init__(self):
        super().__init__('rectif_poses_node')

        # Declare parameters
        # self.declare_parameter('tracker_name', 'vrpn_mocap/marker')
        self.declare_parameter('tracker_name', 'marker')
        # self.declare_parameter('tracker_name_rectified', 'vrpn_mocap/marker_rectified')



        self.tracker_name = self.get_parameter('tracker_name').value



        # Parameters
        tracker_name = self.get_parameter('tracker_name').value
        self.get_logger().info(f"Found tracker with name: {tracker_name}")
        input_topic = "/" + tracker_name + "/pose"
        output_topic = "/" + tracker_name + "/rectified/pose"
        # output_topic = self.get_parameter('tracker_name_rectified').value
        # input_topic = "/vrpn_mocap/Robot02/pose"
        # output_topic = "/vrpn_mocap/Robot02/pose/rotated"

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers and Publishers
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            input_topic,
            self.pose_callback,
            qos_profile
        )
        self.pose_publisher = self.create_publisher(PoseStamped, output_topic, 10)

        self.get_logger().info(f"Subscribed to: {input_topic}")
        self.get_logger().info(f"Publishing to: {output_topic}")


    def rectify_pose(self, pose_in: PoseStamped) -> PoseStamped:
        """
        Switch x, y and z to match robotics coordinate frame.
        Args:
        - pose_in: PoseStamped
        Return:
        - pose_out: PoseStamped
        """

        # copy header
        pose_out = PoseStamped()
        pose_out.header = pose_in.header

        # swap position coordinates
        pose_out.pose.position.x = pose_in.pose.position.z
        pose_out.pose.position.y = pose_in.pose.position.x
        pose_out.pose.position.z = pose_in.pose.position.y

        # swap orientation coordinates
        pose_out.pose.orientation.x = pose_in.pose.orientation.z
        pose_out.pose.orientation.y = pose_in.pose.orientation.x
        pose_out.pose.orientation.z = pose_in.pose.orientation.y
        pose_out.pose.orientation.w = pose_in.pose.orientation.w

        return pose_out


    def pose_callback(self, msg: PoseStamped):
        """
        Subscribes to the pose topic and rectifies the pose,
        by switching x, y and z to correspond to the ROS coordinate frame.
        """
        transformed_pose_msg = self.rectify_pose(msg)
        self.pose_publisher.publish(transformed_pose_msg)


def main(args=None):
    """
    Main function that creates a ROS 2 node with a pose transformer.
    """
    rclpy.init(args=args)
    node = PoseRectifyer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
