import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Bool
from xarmclient import XArm

class Leader(Node):

    def __init__(self):
        super().__init__("gripper_node")
        self.subscription = self.create_subscription(Bool, "/grip", self.listener_callback, 10)
        self.xarm = XArm()

    def listener_callback(self, msg):
        self.xarm.grip(msg.data)
        self.get_logger().info(f"grip I heard: {msg.data}")

def main(args=None):
    try:
        with rclpy.init(args=args):
            node = Leader()
            rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()

