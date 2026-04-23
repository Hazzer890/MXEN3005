# The following imports are necessary
import threading
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from scipy.spatial.transform import Rotation

# Replace the following import with the interface this node is using

from wx250s_interface.action import CartesianPTP
import numpy as np
# You can import here any Python module you plan to use in this node
import time
from xarmclient import XArm
import wx250s_kinematics


class MinimalActionServer(Node):

    def __init__(self):
        super().__init__("cartesian_ptp_node")
        self.goal_handle = None
        self.goal_lock = threading.Lock()
        self.xarm = XArm()
        # Action servers are created using interface type, action name and multiple callback functions
        self.action_server = ActionServer(
            self,
            CartesianPTP,
            "cartesian_ptp_action",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

    def destroy(self):
        self.action_server.destroy()
        super().destroy_node()

    def inverse_kinematics(self, goal_request):
        self.get_logger().info(f"inverse kinematics = cool")

        current_joints = self.xarm.get_joints()
        roll = 0
        pitch = 0
        yaw = 0

        translation = np.array((goal_request.x, goal_request.y, goal_request.z))

        self.get_logger().info(f"inverse kinematics = dumb")
        rot = Rotation.from_euler('XYZ', [roll, pitch, yaw], degrees=True)
        self.get_logger().info(f"inverse kinematics = dumber")
        rotation_matrix = rot.as_matrix()
        self.get_logger().info(f"inverse kinematics = dumbest")

        htm = np.hstack(rotation_matrix, translation.reshape(3,1)) ## I am the problem
        self.get_logger().info(f"inverse kinematics = dumbe!!")

        joints = wx250s_kinematics.ik(current_joints, htm)
        
        if joints == None:
            self.get_logger().info(f"Did Not Converge")
        self.get_logger().info(f"Did Converge")

        return joints
    
    def forward_kinematics(self):
        joint_position_degrees = self.xarm.get_joints()


        htm, _ = wx250s_kinematics.fk(joint_position_degrees)

        translation = htm[:3, 3]

        return translation


    # This function is called whenever new goal request is received
    def goal_callback(self, goal_request):
        self.get_logger().info(f"goal set")

        joint_goal = self.inverse_kinematics(goal_request)
        # Accept or reject a client request to begin an action.
        if self.xarm.is_goal_valid(joint_goal) == 0: ## Fix
            self.get_logger().info("Received valid goal request")
            return GoalResponse.ACCEPT

        self.get_logger().info("Received invalid goal request")
        return GoalResponse.REJECT
        # ... or return GoalResponse.REJECT if goal should be rejected

    # This function is called whenever new goal has been accepted
    def handle_accepted_callback(self, goal_handle):
        with self.goal_lock:
            # This server only allows one goal at a time
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.xarm.set_joints(self.xarm.get_joints()) ## Fix

                self.get_logger().info("Aborting previous goal")
                # Abort the existing goal
                self.goal_handle.abort()
            self.goal_handle = goal_handle

        goal_handle.execute()

    # This function is called whenever cancel request is received
    def cancel_callback(self, goal):
        # Accept or reject a client request to cancel an action.
        self.get_logger().info("Received cancel request")
        self.xarm.set_joints(self.xarm.get_joints())

        return CancelResponse.ACCEPT

    # This function is called at the start of action execution
    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal...")

        target =  self.inverse_kinematics(goal_handle.request)

        result = CartesianPTP.Result()

        feedback_msg = CartesianPTP.Feedback()
        self.xarm.set_joints(target)
        current_value = self.xarm.get_joints()
        prev_value = current_value
        time.sleep(0.2)

        while True:
            current_value = self.xarm.get_joints()
            if not goal_handle.is_active:
                return CartesianPTP.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return CartesianPTP.Result()

            if all(abs(current - target) <= 5
                   for current, target in zip(current_value, target)):
                break
            
            current_position = self.forward_kinematics()
            feedback_msg.x_current = current_position[0]
            feedback_msg.y_current = current_position[1]
            feedback_msg.z_current = current_position[2]

            goal_handle.publish_feedback(feedback_msg)

            for i in range(len(current_value)):
                if abs(current_value[i] - prev_value[i]) < 0.5 and abs(current_value[i] - target[i]) > 5:
                    self.get_logger().info("Stalled out")
                    self.xarm.set_joints(self.xarm.get_joints())

                    goal_handle.abort()
                    

                    return CartesianPTP.Result()

            prev_value = current_value
            time.sleep(0.2)

        with self.goal_lock:
            if not goal_handle.is_active:
                return CartesianPTP.Result()
            goal_handle.succeed()

        result.success = True
        return result



def main(args=None):
    try:
        with rclpy.init(args=args):
            node = MinimalActionServer()

            # We use a MultiThreadedExecutor to handle incoming goal requests concurrently
            executor = MultiThreadedExecutor()
            rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()

