import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_srvs.srv import Trigger
from xarmclient import XArm

class Homing(Node):

    def __init__(self):
        super().__init__("homing_node")
        self.service= self.create_service(Trigger, "/homing", self.listener_callback)
        self.xarm = XArm()

    def listener_callback(self, request, response):
        self.xarm.home()
        self.get_logger().info(f"I heard: {msg.data}")
        response.success = True
        return response

def main(args=None):
    try:
        with rclpy.init(args=args):
            node = Homing()
            rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()

