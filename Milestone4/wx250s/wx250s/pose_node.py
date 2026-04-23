import rclpy
import math
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from wx250s_interface.action import JointPTP
from geometry_msgs.msg import Pose
from xarmclient import XArm

import wx250s_kinematics

from scipy.spatial.transform import Rotation

class Leader(Node):

    def __init__(self):
        super().__init__("joint_state_node")
        self.publisher = self.create_publisher(Pose, "/Pose", 10)
        self.xarm = XArm()
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        pose = Pose()
        joint_position_degrees = self.xarm.get_joints()


        htm, _ = wx250s_kinematics.fk(joint_position_degrees)

        translation = htm[:3, 3]
        rotation_matrix = htm[:3, :3]

        rotation_matrix = Rotation.from_matrix(rotation_matrix)

        pose.position.x = translation[0]
        pose.position.y = translation[1]
        pose.position.z = translation[2]

        quat = rotation_matrix.as_quat()
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        self.publisher.publish(pose)
        # self.get_logger().info(f"Pose: {pose}")
        # self.get_logger().info(f"Trans: {trans}")

def main(args=None):
    try:
        with rclpy.init(args=args):
            node = Leader()
            rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()

