#!/usr/bin/env python

"""
    ir_fp/src/planner/movement_demo.py

    description: demonstration of using the RobotController class

    usage: rosrun ir_fp movement_demo.py
"""

from robot_controller import RobotController, RobotInterruptException

robot = RobotController("movement_demo")

def movement_demo():
    while not robot.is_shutdown():
        robot.drive_forward(2);
        robot.turn_right(180);

if __name__ == "__main__":
    try:
        movement_demo()
        robot.wait()
    except RobotInterruptException:
        pass
