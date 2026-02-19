from xarmclient import XArm
from enum import Enum
import time

# Variables #
tolerance = 1.1

# (WAIST, SHOULDER, ELBOW, FOREARM ROLL, WRIST ANGLE, WRIST ROTATE)
READIED = (0,28,-56,-1,-56,4)
HOVER_LEFT = (-55,0,-34,3,-53,-61)
HIGH_LEFT = (-55,0,0,3,-53,-61)
LEFT = (-56,-7,-35,3,-44.5,-64)
HOVER_RIGHT = (55,-5,-27,-1,-56,57)
HIGH_RIGHT = (55,-5,0,-1,-56,57)
RIGHT = (55,-10,-30,-2,-50,55)

# Functions #
def reachedPosition(target_joints, current_joints):
    if all(abs(current - target) <= tolerance
           for current, target in zip(current_joints, target_joints)):
        return True
    else:
        return False

def attemptMovement(target):
    xarm.set_joints(target)
    while not reachedPosition(target, xarm.get_joints()):
        pass

# Script #
print("Begin")
xarm = XArm()
xarm.home()
xarm.grip(0)
time.sleep(1)
attemptMovement(READIED)
attemptMovement(HIGH_LEFT)
attemptMovement(HOVER_LEFT)
attemptMovement(LEFT)
xarm.grip(1)
time.sleep(1)
attemptMovement(HIGH_LEFT)
attemptMovement(HIGH_RIGHT)
attemptMovement(HOVER_RIGHT)
attemptMovement(RIGHT)
xarm.grip(0)
attemptMovement(HOVER_RIGHT)
attemptMovement(READIED)
xarm.rest()
print("End")
