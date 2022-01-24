import numpy as np
import pybullet as p
import random
import time

from env import SIMEnv

# TIP: Some helper functions you might need.
def move_over_bin(e):
    """Move robot arm over the bin."""
    ########### YOUR CODE STARTS HERE ###########
    raise NotImplementedError
    ########### YOUR CODE ENDS HERE ###########

def move_outside_bin(e):
    """Move robot arm away from the bin."""
    ########### YOUR CODE STARTS HERE ###########
    raise NotImplementedError
    ########### YOUR CODE ENDS HERE ###########


def approach_and_pick(e, position):
    """Approach a block and pick it with the robot arm."""
    ########### YOUR CODE STARTS HERE ###########
    raise NotImplementedError
    ########### YOUR CODE ENDS HERE ###########

def main():

    e = SIMEnv()
    e.robot.go_home()

    while 1:
        ########### YOUR CODE STARTS HERE ###########
        raise NotImplementedError
        ########### YOUR CODE ENDS HERE ###########

if __name__ == "__main__":
    main()
