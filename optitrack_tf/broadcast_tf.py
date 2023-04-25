import rclpy
import math
import numpy as np

from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped


class Optitrack_TF_Publisher(Node):

    def __init__(self):
        super().__init__('optitrack_tf_publisher')
        self.declare_parameter('tracker_name', 'marker')
        self.tracker_name = self.get_parameter('tracker_name').value
        self.pose_topic_name = "/" + self.tracker_name + "/pose"
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_subscriber = self.create_subscription(PoseStamped, self.pose_topic_name, self.listener_callback, 10)
        self.pose_subscriber

    def listener_callback(self, msg):
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
    rclpy.init(args=args)

    pose_subscriber_object = Optitrack_TF_Publisher()

    try:
        rclpy.spin(pose_subscriber_object)
    except KeyboardInterrupt:
        pose_subscriber_object.destroy_node()

    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
