#!/usr/bin/env python3

from robot_controller import RobotController
import rospy
import random


class Spiral:
    def __init__(self, map_width, map_height, grid_size):

        # use this object to control the robot
        self.robot_controller = RobotController()

        # Write any other attributes here
        self.map_width = map_width
        self.map_height = map_height
        self.grid_size = grid_size

    # How to do cells and check if they've been explored?
    def traverse(self):
        # Turn the robots cooridantes into grid X and Y values
        gridX = self.robot_controller.get_x() // self.grid_size
        gridY = self.robot_controller.get_y() // self.grid_size
        # Initialise the direction the robot is facing
        direction = "E"
        # Makes a 2D array out of the map
        grid = [[0] * (self.map_width // self.grid_size)] * (
            self.map_height // self.grid_size
        )
        # Turns current grid position to explored
        grid[gridX][gridY] = 1

        while True:
            if (
                self.robot_controller.is_blocked_in_right()
                and self.check_right_cell(direction, grid, gridX, gridY) == False
            ):
                self.robot_controller.turn_right(90)
                direction = self.rotate_right(direction)
            elif (
                self.robot_controller.is_blocked_in_front()
                and self.check_forward_cell(direction, grid, gridX, gridY) == False
            ):
                self.robot_controller.drive_forward(2)

                if direction == "E":
                    gridX += 1
                elif direction == "S":
                    gridY += 1
                elif direction == "W":
                    gridX -= 1
                elif direction == "N":
                    gridY -= 1

            elif (
                self.robot_controller.is_blocked_in_left()
                and self.check_left_cell(direction, grid, gridX, gridY) == False
            ):
                self.robot_controller.turn_left(90)
                direction = self.rotate_left(direction)
            else:
                self.robot_controller.stop_robot()
                # Move to new location and go again
                break

    # Change direction when rotating right
    def rotate_right(direction):
        if direction == "E":
            direction = "S"
        elif direction == "S":
            direction = "W"
        elif direction == "W":
            direction = "N"
        elif direction == "N":
            direction = "E"
        return direction

    # Change direction when rotating left
    def rotate_left(direction):
        if direction == "E":
            direction = "N"
        elif direction == "S":
            direction = "E"
        elif direction == "W":
            direction = "S"
        elif direction == "N":
            direction = "W"
        return direction

    # Check if right cell is unexplored
    def check_right_cell(direction, grid, x, y):
        if direction == "E":
            if grid[x][y + 1] == 1:
                return True
        elif direction == "S":
            if grid[x - 1][y] == 1:
                return True
        elif direction == "W":
            if grid[x][y - 1] == 1:
                return True
        elif direction == "N":
            if grid[x + 1][y] == 1:
                return True
        return False

    # Check if left cell is unexplored
    def check_left_cell(direction, grid, x, y):
        if direction == "E":
            if grid[x][y - 1] == 1:
                return True
        elif direction == "S":
            if grid[x + 1][y] == 1:
                return True
        elif direction == "W":
            if grid[x][y + 1] == 1:
                return True
        elif direction == "N":
            if grid[x - 1][y] == 1:
                return True
        return False

    # Check if front cell is unexplored
    def check_forward_cell(direction, grid, x, y):
        if direction == "E":
            if grid[x + 1][y] == 1:
                return True
        elif direction == "S":
            if grid[x][y + 1] == 1:
                return True
        elif direction == "W":
            if grid[x - 1][y] == 1:
                return True
        elif direction == "N":
            if grid[x][y - 1] == 1:
                return True
        return False


if __name__ == "__main__":
    try:
        spiral = Spiral(20, 12, 10)

        while not rospy.is_shutdown():
            spiral.traverse()
            break

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
