import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import JointState
from xarmclient import XArm


class JointStateNode(Node):

    def __init__(self):
        super().__init__("joint_state_node")
        self.publisher = self.create_publisher(JointState, "/joint_state", 10)
        self.timer_period = 1 / 20  # seconds (20 Hz)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.xarm = XArm()
        self.get_logger().info("Initialise Joint State Node (radians)")

    def timer_callback(self):
        # xarm reports degrees; RViz / robot_state_publisher expect radians
        joints = np.array(self.xarm.get_joints(), dtype=float) * np.pi / 180.0

        msg = JointState()
        msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        msg.position = joints.tolist()
        self.publisher.publish(msg)


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = JointStateNode()
            rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
