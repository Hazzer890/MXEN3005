import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

import numpy as np
import time

from sensor_msgs.msg import Joy
from xarmclient import XArm

class RobotController(Node):

    def __init__(self):
        super().__init__("robot_controller")
        self.subscription = self.create_subscription(Joy, "/joy", self.listener_callback, 10)
        self.home = False
        self.xarm = XArm()
        self.get_logger().info("Initialise Robot Controller")

        self.gameController = Joy()

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.current_joints = np.array(self.xarm.get_joints(), dtype=float)
        self.target_joints = self.current_joints.copy()

        self.piCo = PIController(
            kp=np.array([0.8, 0.8, 0.8, 0.5, 0.5, 0.5]),
            ki=np.array([0.05, 0.05, 0.05, 0.02, 0.02, 0.02]),
            dt=self.timer_period,
            output_limit=np.array([2.0, 2.0, 2.0, 1.0, 1.0, 1.0]),
            integral_limit=np.array([10.0, 10.0, 10.0, 5.0, 5.0, 5.0])
        )

        self.jog_step = np.array([5.0, 5.0, 5.0, 5.0, 5.0, 5.0])

    def listener_callback(self, msg, response):
        # Instantiate gameController Inputs
        self.gameController.axes = msg.axes
        self.gameController.buttons = msg.buttons
        return response

    def timer_callback(self):
        # Homing Arm
        should_home = self.gameController.buttons[10] == 1
        if (should_home == True and self.home == False): # is there a pythonic way to do this?
            self.get_logger().info("Home Robot Joints")
            self.xarm.home()
            time.sleep(2)
            self.current_joints = np.array(self.xarm.get_joints(), dtype=float)
            self.target_joints = self.current_joints.copy()
            self.piCo.reset()
        self.home = should_home

        # Joint Mode
        # Joint 2 Axes Conversion
        J1 = self.gameController.axes[3]
        J2 = self.gameController.axes[4]
        J3 = self.gameController.axes[1]
        J4 = self.gameController.axes[0]
        J5 = self.gameController.axes[7]
        J6 = self.gameController.axes[6]
        joystick_cmd = np.array([J1, J2, J3, J4, J5, J6], dtype=float)

        # Deadzone
        joystick_cmd[np.abs(joystick_cmd) < 0.1] = 0.0

        # Update target joints from joystick commands
        self.target_joints += joystick_cmd * self.jog_step * self.timer_period

        # store current robot joints
        self.current_joints = np.array(self.xarm.get_joints(), dtype=float)

        # PI correction multiplier
        piMult = self.piCo.update(
            target=self.target_joints,
            measurement=self.current_joints
        )

        # Commanded joint position
        commanded_joints = self.current_joints + piMult

        self.get_logger().info(
            f"target={self.target_joints}, current={self.current_joints}, cmd={commanded_joints}"
        )

        self.xarm.set_joints(commanded_joints, "high_acc")

        #End Timer Callback


class PIController:
    def __init__(self, kp, ki, dt, output_limit=None, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.dt = dt

        self.output_limit = output_limit
        self.integral_limit = integral_limit

        self.integral = np.zeros(6)

    def reset(self):
        self.integral[:] = 0.0

    def update(self, target, measurement):
        error = target - measurement

        self.integral += error * self.dt

        if self.integral_limit is not None:
            self.integral = np.clip(
                self.integral,
                -self.integral_limit,
                self.integral_limit
            )

        output = self.kp * error + self.ki * self.integral

        if self.output_limit is not None:
            output = np.clip(
                output,
                -self.output_limit,
                self.output_limit
            )

        return output

def main(args=None):
    try:
        with rclpy.init(args=args):
            node = RobotController()
            rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
