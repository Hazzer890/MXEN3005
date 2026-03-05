import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from random import uniform

class Leader(Node):

    def __init__(self):
        super().__init__("leader")
        self.publisher = self.create_publisher(Twist, "/arena/leader/cmd_vel", 10)
        self.subscription = self.create_subscription(Bool, "/turbo", self.listener_callback, 10)
        timer_period = 0.5  # seconds
        self.x_vel = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        twist = Twist()
        twist.linear.x = self.x_vel
        twist.angular.z = uniform(-3.14, 3.14)
        self.publisher.publish(twist)
        self.get_logger().info(f"Publishing: {twist}")

    def listener_callback(self, msg):
        if msg.data:
            self.x_vel = 2.0
        else:
            self.x_vel = 1.0
        self.get_logger().info(f"I heard: {msg.data}")

def main(args=None):
    try:
        with rclpy.init(args=args):
            node = Leader()
            rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()

