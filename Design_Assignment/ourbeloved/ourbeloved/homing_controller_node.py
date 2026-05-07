import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Joy
from xarmclient import XArm

class Homing_Controller(Node):

    def __init__(self):
        super().__init__("homing_controller_node")
        self.subscription = self.create_subscription(Joy, "/joy", self.listener_callback, 10)
        self.home = False
        self.xarm = XArm()
        self.get_logger().info("HEY LOL")

    def listener_callback(self, msg, response):
        self.get_logger().info(f"l")
        should_home = msg.buttons[10] == 1

        self.get_logger().info(f"{should_home}")
        
        if (should_home == True and self.home == False):
            self.xarm.home()
        self.home = should_home
        return response

def main(args=None):
    try:
        with rclpy.init(args=args):
            node = Homing_Controller()
            rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
