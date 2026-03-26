import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_srvs.srv import SetBool
from xarmclient import XArm

class Gripper(Node):

    def __init__(self):
        super().__init__("gripper_node")
        self.service= self.create_service(SetBool, "/gripper", self.listener_callback)
        self.xarm = XArm()

    def listener_callback(self, msg, response):
        self.xarm.grip(msg.data)
        # self.get_logger().info(f"I heard: {msg.data}")
        response.success = True
        return response

def main(args=None):
    try:
        with rclpy.init(args=args):
            node = Gripper()
            rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
