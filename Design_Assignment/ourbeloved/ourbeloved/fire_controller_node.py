import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

class Fire_Controller(Node):

    def __init__(self):
        super().__init__("fire_controller_node")
        self.subscription = self.create_subscription(Joy, "/joy", self.listener_callback, 10)
        self.publisher = self.create_publisher(Bool, "/fire", 10)
        self.fire = False

    def listener_callback(self, msg):
        should_fire = msg.axes[5] < 0

        if (should_fire == False and self.fire == True):
            bool_publish = Bool()
            bool_publish.data = False
            self.publisher.publish(bool_publish)
        
        
        if (should_fire == True and self.fire == False):
            bool_publish = Bool()
            bool_publish.data = True
            self.publisher.publish(bool_publish)
        
        self.fire = should_fire

def main(args=None):
    try:
        with rclpy.init(args=args):
            node = Fire_Controller()
            rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
