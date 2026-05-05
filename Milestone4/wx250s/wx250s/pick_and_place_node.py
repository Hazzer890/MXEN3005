import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import time

from wx250s_interface.srv import PickAndPlace
# from wx250s_interface.action import JointPTP
from xarmclient import XArm
import numpy as np
from scipy.spatial.transform import Rotation
import wx250s_kinematics
from xarmclient import XArm

def calculate_intermediate_targets(self, x, y, z):
    self.intermediate_targets = []
    max_step_size = 30 #mm

    current_position = forward_kinematics(self)
    dx = x - current_position[0]
    dy = y - current_position[1]
    dz = z - current_position[2]

    distance = np.square(dx) + np.square(dy) + np.square(dz)
    distance = np.sqrt(distance)

    steps_to_take = np.ceil(distance / max_step_size)
    step_size = distance / steps_to_take

    current_joints = self.xarm.get_joints()

    for step in range(round(steps_to_take)):
        new_x = current_position[0] + dx / steps_to_take * (step + 1) # Doesn't include current pos, does includefinal pos
        new_y = current_position[1] + dy / steps_to_take * (step + 1) # Doesn't include current pos, does includefinal pos
        new_z = current_position[2] + dz / steps_to_take * (step + 1) # Doesn't include current pos, does includefinal pos
        
        new_joints = inverse_kinematics(self, new_x, new_y, new_z, current_joints)
        if new_joints is None:
            return False
        if self.xarm.is_goal_valid(new_joints) != 0:
            self.get_logger().info(f"Received invalid position request")
            return False
        
        current_joints = new_joints
        self.intermediate_targets.append(new_joints)

    return True

def inverse_kinematics(self, x, y, z, current_joints):

    roll = 0
    pitch = -90
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
class PickAndPlaceNode(Node):

    def __init__(self):
        super().__init__("pick_and_place")
        self.service= self.create_service(PickAndPlace, "/pick_and_place", self.listener_callback)
        self.xarm = XArm()
        self.intermediate_targets = []

    def move_to_point(self, x, y, z):
        self.get_logger().info(f"Going to: {x}, {y}, {z}")
 
        if calculate_intermediate_targets(self, x, y, z) == False:
            self.get_logger().info(f"Can't go to: {x}, {y}, {z}")
            return False


        current_joints = self.xarm.get_joints()
        prev_value = current_joints

        tolerance_degrees = 2

        current_joints = self.xarm.get_joints()
        i = self.intermediate_targets
        self.get_logger().info(f"int targets: {i}")
        target_joints = self.intermediate_targets[-1]
        self.xarm.set_joints(target_joints, "high_acc")

        while not all(abs(current - target) <= tolerance_degrees
                for current, target in zip(current_joints, target_joints)):            



            # for i in range(len(current_joints)):
            #     if abs(current_joints[i] - prev_value[i]) < 0.5 and abs(current_joints[i] - current_target_joints[i]) > tolerance_degrees:
            #         self.get_logger().info("Stalled out")
            #         self.get_logger().info(f"current joints - {current_joints[i]}")
            #         self.get_logger().info(f"prev joints - {prev_value[i]}")
            #         self.get_logger().info(f"target joints - {current_target_joints[i]}")
            #         self.xarm.set_joints(self.xarm.get_joints())

            #         goal_handle.abort()
    
                    

            #         return False

            prev_value = current_joints
            current_joints = self.xarm.get_joints()
            time.sleep(0.02)
        return True




    def listener_callback(self, msg, response):
        # above pick
        if self.move_to_point(msg.xpick, msg.ypick, 150) == False:
            self.get_logger().info(f"I give up")

        # pick
        if self.move_to_point(msg.xpick, msg.ypick, 50) == False:
            self.get_logger().info(f"I give up")
        
        self.xarm.grip(True)

        # above pick
        if self.move_to_point(msg.xpick, msg.ypick, 150) == False:
            self.get_logger().info(f"I give up")
        
        # above place
        if self.move_to_point(msg.xplace, msg.yplace, 150) == False:
            self.get_logger().info(f"I give up")
        
        # place
        if self.move_to_point(msg.xplace, msg.yplace, 50) == False:
            self.get_logger().info(f"I give up")
        self.xarm.grip(False)

        # above place
        if self.move_to_point(msg.xplace, msg.yplace, 150) == False:
            self.get_logger().info(f"I give up")
        # response.success = True
        return response

def main(args=None):
    try:
        with rclpy.init(args=args):
            node = PickAndPlaceNode()
            rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
