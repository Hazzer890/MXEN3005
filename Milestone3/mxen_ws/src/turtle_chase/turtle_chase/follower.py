#follower
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from math import atan2

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from turtlesim_msgs.msg import Pose

from random import uniform

class Follower(Node):

    def __init__(self):
        super().__init__("follower")
        self.publisher = self.create_publisher(Twist, "/arena/follower/cmd_vel", 10)
        self.subscription = self.create_subscription(Pose, "/arena/leader/pose", self.listener_callback, 10)
        self.subscription2 = self.create_subscription(Pose, "/arena/follower/pose", self.listener_callback2, 10)
        timer_period = 0.5  # seconds
        self.x_vel = 1.0
        self.leader_pose = Pose()
        self.follower_pose = Pose()
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        twist = Twist()
        twist.linear.x = 1.0
        twist.angular.z = atan2((self.leader_pose.y - self.follower_pose.y), (self.leader_pose.x - self.follower_pose.x)) - self.follower_pose.theta
        self.publisher.publish(twist)
        self.get_logger().info(f"Publishing: {twist}")

    def listener_callback(self, msg):
        self.leader_pose = msg

    def listener_callback2(self, msg):
        self.follower_pose = msg
        self.get_logger().info(f"I saw: {msg}")

def main(args=None):
    try:
        with rclpy.init(args=args):
            node = Follower()
            rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()

