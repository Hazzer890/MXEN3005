import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

import numpy as np
import time

from sensor_msgs.msg import Joy, JointState
from xarmclient import XArm
from wx250s_kinematics import fk, ik


class RobotController(Node):

    def __init__(self):
        super().__init__("robot_controller")
        self.subscription = self.create_subscription(Joy, "/joy", self.listener_callback, 10)
        self.precise_sub = self.create_subscription(
            JointState, "/precise_joints", self.precise_callback, 10
        )
        self.home = False
        self.xarm = XArm()
        self.get_logger().info("Initialise Robot Controller")

        self.gameController = Joy()

        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.currentrent_joints = np.array(self.xarm.get_joints(), dtype=float)
        self.target_joints = self.currentrent_joints.copy()
        self.commanded_joints = self.currentrent_joints.copy()

        self.piCo = PIController(
            kp=np.array([0.8, 0.8, 0.8, 0.5, 0.5, 0.5]),
            ki=np.array([0.05, 0.05, 0.05, 0.02, 0.02, 0.02]),
            dt=self.timer_period,
            output_limit=np.array([2.0, 2.0, 2.0, 1.0, 1.0, 1.0]),
            integral_limit=np.array([10.0, 10.0, 10.0, 5.0, 5.0, 5.0])
        )

        self.jog_step = np.array([9.0, 9.0, 9.0, 9.0, 9.0, 9.0])
        self.deadzone = 0.1
        self.started = False

        # Control-mode state machine ('joint' 'cartesian' 'attack')
        self.control_mode = 'joint'
        self.drift = False
        self.tune_vel = [40] * 6

        # Cartesian reference
        self.ref_htm, _ = fk(self.xarm.get_joints())

        # Precise-positioning 
        self.within_tolerance = True
        self.within_count = 0
        self.settled_count = 0
        self.tolerance = 0.1
        self.phase = 'initial'  # ()'initial' 'moving' 'pi')
        self.precise_goal = self.currentrent_joints.copy()
        self.prev_joints = self.currentrent_joints.copy()

        self.WITHIN_CYCLES = 5
        self.SETTLE_CYCLES = 5

    def listener_callback(self, msg):
        # Instantiate gameController Inputs
        self.gameController.axes = msg.axes
        self.gameController.buttons = msg.buttons
        self.started = True

    def precise_callback(self, msg):
        goal = tuple(msg.position)
        if self.xarm.is_goal_valid(goal) == 0:
            self.within_tolerance = False
            self.within_count = 0
            self.settled_count = 0
            self.phase = 'initial'
            self.control_mode = 'joint'
            self.precise_goal = np.array(msg.position, dtype=float)
            self.prev_joints = np.array(self.xarm.get_joints(), dtype=float)
            self.piCo.reset()
            self.get_logger().info(f"Precise goal accepted: {self.precise_goal}")
        else:
            self.get_logger().info(
                f"Rejected invalid precise goal {msg.position}"
            )

    def timer_callback(self):
        if not self.started:
            return

        # Homing Arm
        should_home = self.gameController.buttons[10] == 1
        if (should_home is True and self.home is False):
            self.get_logger().info("Home Robot Joints")
            self.xarm.home()
            time.sleep(2)
            self.currentrent_joints = np.array(self.xarm.get_joints(), dtype=float)
            self.commanded_joints = self.currentrent_joints.copy()
            self.piCo.reset()
            self.within_tolerance = True
        self.home = should_home

        # Control-mode 
        if self.gameController.buttons[8] == 1:
            if self.control_mode != 'cartesian':
                self.get_logger().info("Cartesian Mode")
            self.control_mode = 'cartesian'
            self.ref_htm, _ = fk(self.xarm.get_joints())
        if self.gameController.buttons[9] == 1:
            if self.control_mode != 'joint':
                self.get_logger().info("Joint Mode")
            self.control_mode = 'joint'
        if self.gameController.buttons[0] == 1:
            if self.control_mode != 'attack':
                self.get_logger().info("Attack Mode")
            self.control_mode = 'attack'

        if not self.within_tolerance:
            if self.preciseControl():
                return

        # teleop
        self.manualControl()

    def stickActive(self):
        axes = self.gameController.axes
        mapped = np.array(
            [axes[0], axes[1], axes[3], axes[4], axes[6], axes[7]], dtype=float
        )
        return bool(np.any(np.abs(mapped) > self.deadzone))

    def preciseControl(self):
        # Stick nudge aborts
        if self.stickActive():
            self.within_tolerance = True
            self.get_logger().info("Precise positioning aborted by stick input")
            return False

        current = np.array(self.xarm.get_joints(), dtype=float)
        errors = self.precise_goal - current
        failing = np.abs(errors) > self.tolerance
        self.get_logger().info(f"precise errors={np.round(errors, 3)} phase={self.phase}")

        if not np.any(failing):
            self.within_count += 1
            if self.within_count >= self.WITHIN_CYCLES:
                self.within_tolerance = True
                self.get_logger().info("Goal reached within tolerance")
                return False
            return True

        self.within_count = 0

        if self.phase == 'initial':
            self.xarm.set_joints(tuple(self.precise_goal), "high_acc", velocities=[30] * 6)
            self.phase = 'moving'

        elif self.phase == 'moving':
            movement = np.abs(current - self.prev_joints)
            if np.all(movement <= 0.2):
                self.settled_count += 1
                if self.settled_count >= self.SETTLE_CYCLES:
                    self.phase = 'pi'
                    self.settled_count = 0
                    self.piCo.reset()
            else:
                self.settled_count = 0
            self.prev_joints = current

        elif self.phase == 'pi':
            piMult = self.piCo.update(target=self.precise_goal, measurement=current)
            commanded = self.precise_goal + piMult
            self.xarm.set_joints(tuple(commanded), "high_acc", velocities=[30] * 6)

        return True

    def manualControl(self):
        if self.stickActive():
            if self.control_mode == 'cartesian':
                target = self.cartesianControl()
            elif self.control_mode == 'attack':
                target = self.attackControl()
            else:
                target = self.jointControl()

            if target is None:
                # Unreachable IK step
                return

            if self.xarm.is_goal_valid(tuple(target)) == 0:
                self.commanded_joints = np.array(target, dtype=float)
                if self.control_mode == 'cartesian':
                    self.xarm.set_joints(tuple(self.commanded_joints))
                else:
                    self.xarm.set_joints(
                        tuple(self.commanded_joints), "high_acc",
                        velocities=self.tune_vel
                    )
            else:
                self.get_logger().info(
                    f"Rejected invalid target {np.round(target, 2)}"
                )
            self.drift = True
        else:
            self.currentrent_joints = np.array(self.xarm.get_joints(), dtype=float)
            if self.drift:
                self.xarm.set_joints(tuple(self.currentrent_joints))
                self.commanded_joints = self.currentrent_joints.copy()
                self.drift = False

    def jointControl(self):
        # joint mapping 
        J1 = self.gameController.axes[3]
        J2 = self.gameController.axes[4]
        J3 = self.gameController.axes[1]
        J4 = self.gameController.axes[0]
        J5 = self.gameController.axes[7]
        J6 = self.gameController.axes[6]
        joystick_cmd = np.array([J1, J2, J3, J4, J5, J6], dtype=float)

        self.currentrent_joints = np.array(self.xarm.get_joints(), dtype=float)
        target = self.currentrent_joints.copy()
        for ii, axis in enumerate(joystick_cmd):
            if np.abs(axis) > self.deadzone:
                target[ii] = axis * self.jog_step[ii] + self.currentrent_joints[ii]

        self.get_logger().info(
            f"currentrent={np.round(self.currentrent_joints, 2)} "
            f"cmd={np.round(target, 2)}"
        )
        return target

    def cartesianControl(self):
        diff_x = self.gameController.axes[4]
        diff_y = self.gameController.axes[3]
        diff_z = self.gameController.axes[1]

        present_htm, _ = fk(self.commanded_joints)
        goal_htm = present_htm.copy()

        rot_mat = self.ref_htm[0:3, 0:3]
        rot_diff = rot_mat @ np.array([diff_x, diff_y, diff_z])
        goal_htm[0:3, 3] += rot_diff

        return self.ikIntermediateStep(present_htm, goal_htm)

    def ikIntermediateStep(self, present_htm, goal_htm):
        final = goal_htm[0:3, 3].copy()
        initial = present_htm[0:3, 3].copy()

        intermediate_htms = []
        angles = self.commanded_joints
        done = False

        while not done:
            done = True
            for k in range(3):
                if abs(final[k] - initial[k]) > 2:
                    done = False
                    initial[k] += 2 * (final[k] - initial[k]) / abs(final[k] - initial[k])
            if not done:
                step_htm = goal_htm.copy()
                step_htm[0:3, 3] = initial
                intermediate_htms.append(step_htm)

        intermediate_htms.append(goal_htm.copy())

        for htm in intermediate_htms:
            angles = ik(angles, htm)
            if angles is None:
                self.get_logger().info("Unreachable IK step, holding position")
                return None

        return angles

    def attackControl(self):
        # game-style
        current = np.array(self.xarm.get_joints(), dtype=float)
        joint0 = self.gameController.axes[3] + current[0]
        joint4 = self.gameController.axes[4] + current[4]
        return np.array([joint0, 70.0, -20.0, 0.0, joint4, 0.0], dtype=float)


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
