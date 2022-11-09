
#resource:https://www.theconstructsim.com/learn-tf2-ros2-concepts-in-practice/
from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

import tf_transformations

from turtlesim.msg import Pose


class FramePublisher(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_publisher')

        # Declare and acquire `turtlename` parameter
        self.declare_parameter('turtlename', 'turtle')
        self.turtlename = self.get_parameter(
            'turtlename').get_parameter_value().string_value

        # Initialize the transform broadcaster
        self.br = TransformBroadcaster(self)

        # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        # callback function on each message
        self.subscription = self.create_subscription(
            Pose,
            f'/{self.turtlename}/pose',
            self.handle_turtle_pose,
            1)

    def handle_turtle_pose(self, msg):
        t = TransformStamped()
        t2 = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.turtlename
        # turtle/drone
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = self.turtlename
        t2.child_frame_id = 'drone'

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg.x - (11.08888/2)
        t.transform.translation.y = msg.y - (11.08888/2)
        t.transform.translation.z = 0.0

        t2.transform.translation.x = -0.5
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 1.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = tf_transformations.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0

        # Send the transformation
        self.br.sendTransform(t)
        self.br.sendTransform(t2)


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()