#!/usr/bin/env python3

from robot_controller import RobotController, RobotInterruptException

ROW_SEPARATION = 0.5

robot = RobotController("zigzag")

def zigzag():
    while not robot.is_shutdown():
        done = True

        robot.drive_until_blocked()
        robot.turn_left(90)

        if not robot.is_blocked_front():
            done = False
            robot.drive_forward(ROW_SEPARATION)

        robot.turn_left(90)
        robot.drive_until_blocked()
        robot.turn_right(90)

        if not robot.is_blocked_front():
            done = False
            robot.drive_forward(ROW_SEPARATION)
            robot.turn_right(90)

        if done:
            break


if __name__ == "__main__":
    try:
        zigzag()
        robot.wait()
    except RobotInterruptException:
        pass
