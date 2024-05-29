"""
This node listens to the pose topic published by the vrpn_mocap
node and republishes the data as TF message.
"""
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped



class OptitrackTFPublisher(Node):
    """
    This class creates the Topic Subscriber and
    the TF Broadcaster for the Pose messages.
    """

    def __init__(self):
        """
        Initiates the pose_subscriber and tf_broadcaster.
        """
        super().__init__('optitrack_tf_publisher')
        self.declare_parameter('tracker_name', 'marker')
        self.tracker_name = self.get_parameter('tracker_name').value
        self.pose_topic_name = "/" + self.tracker_name + "/pose"

        # Define the custom QoS profile to fit with vrpn_mocap 1.1.0
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_subscriber = self.create_subscription(
                PoseStamped,
                self.pose_topic_name,
                self.listener_callback,
                qos_profile
        )

    def listener_callback(self, msg):
        """
        Callback function of the pose_subscriber.
        It publishes the tf messages by the tf_broadcaster.
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.tracker_name

        # Populate tf message
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation.x = msg.pose.orientation.x
        t.transform.rotation.y = msg.pose.orientation.y
        t.transform.rotation.z = msg.pose.orientation.z
        t.transform.rotation.w = msg.pose.orientation.w
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    """
    Main function that initiates the TF Broadcaster Object
    and keeps the node spinning.
    """
    rclpy.init(args=args)

    pose_subscriber_object = OptitrackTFPublisher()

    try:
        rclpy.spin(pose_subscriber_object)
    except KeyboardInterrupt:
        pose_subscriber_object.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
