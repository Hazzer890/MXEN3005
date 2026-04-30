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
        self.intermediate_targets = []

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

    def inverse_kinematics(self, x, y, z, current_joints):

        roll = 0
        pitch = 0
        yaw = 0

        translation = np.array((x, y, z))

        rot = Rotation.from_euler('XYZ', [roll, pitch, yaw], degrees=True)
        rotation_matrix = rot.as_matrix()
        
        htm = np.zeros((3, 4))
        htm[:3, :3] = rotation_matrix
        htm[:3, -1] = translation

        joints = wx250s_kinematics.ik(current_joints, htm)
        
        if joints is None:
            self.get_logger().info(f"Did Not Converge")

        return joints
    
    def forward_kinematics(self):
        joint_position_degrees = self.xarm.get_joints()


        htm, _ = wx250s_kinematics.fk(joint_position_degrees)

        translation = htm[:3, 3]

        return translation


    # This function is called whenever new goal request is received
    def goal_callback(self, goal_request):
        self.get_logger().info(f"goal set")

        self.calculate_intermediate_targets(goal_request.x, goal_request.y, goal_request.z)

        current_joints = self.xarm.get_joints()

        for target in self.intermediate_targets:
            joint_goal = self.inverse_kinematics(target[0], target[1], target[2], current_joints)
            # Accept or reject a client request to begin an action.
            if self.xarm.is_goal_valid(joint_goal) != 0: 

                self.get_logger().info("Received invalid goal request")
                return GoalResponse.REJECT

            current_joints = joint_goal
        self.get_logger().info("Received valid goal request")
        # ... or return GoalResponse.REJECT if goal should be rejected
        return GoalResponse.ACCEPT

    # This function is called whenever new goal has been accepted
    def handle_accepted_callback(self, goal_handle):
        with self.goal_lock:
            # This server only allows one goal at a time
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.xarm.set_joints(self.xarm.get_joints(), "high_acc") ## Fix

                self.get_logger().info("Aborting previous goal")
                # Abort the existing goal
                self.goal_handle.abort()
            self.goal_handle = goal_handle

        goal_handle.execute()

    # This function is called whenever cancel request is received
    def cancel_callback(self, goal):
        # Accept or reject a client request to cancel an action.
        self.get_logger().info("Received cancel request")
        self.xarm.set_joints(self.xarm.get_joints(), "high_acc")

        return CancelResponse.ACCEPT

    def calculate_intermediate_targets(self, x, y, z):
        self.intermediate_targets = []
        max_step_size = 30 #mm

        current_position = self.forward_kinematics()
        dx = x - current_position[0]
        dy = y - current_position[1]
        dz = z - current_position[2]

        distance = np.square(dx) + np.square(dy) + np.square(dz)
        distance = np.sqrt(distance)

        steps_to_take = np.ceil(distance / max_step_size)
        step_size = distance / steps_to_take

        for step in range(round(steps_to_take)):
            new_x = current_position[0] + dx / steps_to_take * (step + 1) # Doesn't include current pos, does includefinal pos
            new_y = current_position[1] + dy / steps_to_take * (step + 1) # Doesn't include current pos, does includefinal pos
            new_z = current_position[2] + dz / steps_to_take * (step + 1) # Doesn't include current pos, does includefinal pos
            

            self.intermediate_targets.append([new_x, new_y, new_z])

    # This function is called at the start of action execution
    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal...")

 

        current_step = 0

        result = CartesianPTP.Result()

        feedback_msg = CartesianPTP.Feedback()
        current_joints = self.xarm.get_joints()
        prev_value = current_joints

        tolerance_degrees = 2

        for current_target in self.intermediate_targets:
            

            current_joints = self.xarm.get_joints()
            current_target_joints = self.inverse_kinematics(current_target[0], current_target[1], current_target[2], self.xarm.get_joints())
            self.xarm.set_joints(current_target_joints, "high_acc")

            while not all(abs(current - target) <= tolerance_degrees
                   for current, target in zip(current_joints, current_target_joints)):
                if not goal_handle.is_active:
                    return CartesianPTP.Result()

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return CartesianPTP.Result()

                
                
                current_position = self.forward_kinematics()
                feedback_msg.x_current = current_position[0] - self.intermediate_targets[-1][0]
                feedback_msg.y_current = current_position[1] - self.intermediate_targets[-1][1]
                feedback_msg.z_current = current_position[2] - self.intermediate_targets[-1][2]

                goal_handle.publish_feedback(feedback_msg)



                # for i in range(len(current_joints)):
                #     if abs(current_joints[i] - prev_value[i]) < 0.5 and abs(current_joints[i] - current_target_joints[i]) > tolerance_degrees:
                #         self.get_logger().info("Stalled out")
                #         self.get_logger().info(f"current joints - {current_joints[i]}")
                #         self.get_logger().info(f"prev joints - {prev_value[i]}")
                #         self.get_logger().info(f"target joints - {current_target_joints[i]}")
                #         self.xarm.set_joints(self.xarm.get_joints())

                #         goal_handle.abort()
                        

                #         return CartesianPTP.Result()

                prev_value = current_joints
                current_joints = self.xarm.get_joints()
                time.sleep(0.02)

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

