import rclpy
import math
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from wx250s_interface.action import JointPTP
from sensor_msgs.msg import JointState
from xarmclient import XArm

class Leader(Node):

    def __init__(self):
        super().__init__("joint_state_node")
        self.publisher = self.create_publisher(JointState, "/joint_state", 10)
        self.xarm = XArm()
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        joint = JointState()
        joint_position_degrees = self.xarm.get_joints()
        joint.position = type(xs)(math.radians(x) for x in joint_position_degrees)
        self.publisher.publish(joint)
        # self.get_logger().info(f"Publishing: {joint}")

def main(args=None):
    try:
        with rclpy.init(args=args):
            node = Leader()
            rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()

