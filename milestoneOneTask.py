from xarmclient import XArm
from enum import Enum
from time import sleep

# Variables #
tolerance = 1.1

class position(Enum):
    # (YAW, SHOULDER, ELBOW, WRIST_ROTATE, WRIST_BWK_ROT, WRIST_FWD_ROT)
    READIED = (0,0,0,0,0,0)
    HOVER_LEFT = (0,0,0,0,0,0)
    LEFT = (0,0,0,0,0,0)
    HOVER_RIGHT = (0,0,0,0,0,0)
    RIGHT = (0,0,0,0,0,0)

# Functions #
def reachedPosition(target_joints, current_joints):
    if all(abs(current - target) <= tolerance
           for current, target in zip(current_joints, target_joints)):
        return True
    else
        return False

def attemptMovement(target):
    while not reachedPosition(target, xarm.get_joints()):
        xarm.set_joints(target)

# Script #
print("Begin")
xarm.home()
xarm.grip(0)
time.sleep(1)
attemptMovement(position.READIED)
attemptMovement(position.HOVER_LEFT)
attemptMovement(position.LEFT)
xarm.grip(1)
time.sleep(1)
attemptMovement(position.HOVER_RIGHT)
attemptMovement(position.RIGHT)
xarm.grip(0)
attemptMovement(position.HOVER_RIGHT
attemptMovement(position.READIED)
print("End")
