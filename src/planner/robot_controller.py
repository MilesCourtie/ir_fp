#!/usr/bin/env python3

"""
    ir_fp/src/planner/robot_controller.py

    description: interface for controlling the robot in ROS Stage

    usage: instantiate the RobotController class in your ROS node
"""

import math
import rospy
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class RobotController:

    def __init__(self):

        # ROS node initialisation
        rospy.init_node("robot_controller", anonymous=True)

        # publisher for sending velocity commands
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # subscriber for receiving pose estimates
        rospy.Subscriber("/odom", Odometry, self.update_pose)

        # subscriber for receiving laser scan data
        rospy.Subscriber("/base_scan", LaserScan, self.update_scan)

        # the most recent laser scan data
        self.scan = None

        # the most recent pose estimate
        self.x = 0
        self.y = 0
        self.theta = 0

        # desired rate at which to publish velocity commands
        self.rate = rospy.Rate(10)

        self.PIXEL_TO_CM = 2 # unused

        # tolerances for the robot's estimated pose being equal to a desired pose
        self.DISTANCE_TOLERANCE = 0.1 # metres
        self.ROTATION_TOLERANCE = 0.001 # radians

        # forwards/backwards speed
        self.LINEAR_VEL = 1 # metres per ???

        # turning speed
        self.ANGULAR_VEL = math.radians(30) # radians per ???

        # distance from the range sensor for an obstacle to register as 'blocking'
        # the robot
        self.IS_BLOCKED_READING = 0.7 # metres


    def update_pose(self, data):
        """
            Callback for when a new pose estimate is received.
            Updates self.x, self.y and self.theta from given estimate.
        """
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        quaternion = data.pose.pose.orientation
        (_, _, self.theta) = euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        )


    def get_x(self):
        """
            Returns the most recent estimate of the robot's X position.
        """
        return self.x


    def get_y(self):
        """
            Returns the most recent estimate of the robot's Y position.
        """
        return self.y


    def update_scan(self, data):
        """
            Callback for when new laser scan data is received.
            Updates self.scan from the received data.
        """
        self.scan = data


    def is_blocked_in_right(self):
        """
            Returns true iff the robot is obstructed on the right-hand side.
        """
        self.rate.sleep()
        for i in range(166):
            if self.scan.ranges[i] < self.IS_BLOCKED_READING:
                return True
        return False


    def is_blocked_in_front(self):
        """
            Returns true iff the robot is obstructed from in front.
        """
        self.rate.sleep()
        for i in range(166, 334):
            if self.scan.ranges[i] < self.IS_BLOCKED_READING:
                return True
        return False


    def is_blocked_in_left(self):
        """
            Returns true iff the robot is obstructed on the left-hand side.
        """
        self.rate.sleep()
        for i in range(334, len(self.scan.ranges)):
            if self.scan.ranges[i] < self.IS_BLOCKED_READING:
                return True
        return False


    def euclidean_distance(self, x, y):
        """
            Returns the Euclidean distance between the most recent pose
            estimate and a given XY coordinate.
        """
        return math.sqrt(pow((x - self.x), 2) + pow((y - self.y), 2))


    def steering_angle(self, x, y):
        """
            Returns the angle in radians, between -Pi and Pi, between the
            positive X-axis and the vector from the most recent pose
            estimate to a given XY coordinate.
        """
        return math.atan2(y - self.y, x - self.x)


    def stop_robot(self):
        """
            Publishes a velocity command to stop the robot.
        """
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)


    def drive_forward(self, distance):
        """
            Publishes velocity commands to drive the robot forwards until
            it has moved a specified distance, then stops the robot.
        """
        self.rate.sleep()

        x = self.x
        y = self.y

        vel_msg = Twist()

        while distance > self.DISTANCE_TOLERANCE:

            vel_msg.linear.x = self.LINEAR_VEL
            self.velocity_publisher.publish(vel_msg)

            walked_distance = self.euclidean_distance(x, y)
            x = self.x
            y = self.y
            distance -= walked_distance

            self.rate.sleep()

        self.stop_robot()


    def rotate(self, angle, direction):
        """
            Publishes velocity commands to rotate the robot in a given
            direction until it has turned a given angle, then stops the
            robot.
            
            params:
                angle : amount to turn in degrees
                direction : "l" to turn left (clockwise) or "r" to turn right
        """
        self.rate.sleep()

        if angle == 0:
            return

        if direction == "l":
            rotation_direction = 1
        elif direction == "r":
            rotation_direction = -1

        print("current theta: " + str(math.degrees(self.theta)))

        vel_msg = Twist()

        # calculate desired orientation in radians
        angle = (rotation_direction * math.radians(angle)) + self.theta

        # restrict desired orientation to be between -pi and pi
        if abs(math.degrees(angle)) > 180:
            angle = rotation_direction * math.radians(abs(math.degrees(angle)) - 360)

        print("heading to go: " + str(math.degrees(angle)))

        if abs(angle - self.theta) <= self.ROTATION_TOLERANCE:
            while abs(angle - self.theta) <= self.ROTATION_TOLERANCE:
                vel_msg.angular.z = self.ANGULAR_VEL * rotation_direction * 0.5
                self.velocity_publisher.publish(vel_msg)
                self.rate.sleep()

        while abs(angle - self.theta) > self.ROTATION_TOLERANCE:
            vel_msg.angular.z = abs(angle - self.theta) * rotation_direction * 0.5
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        self.stop_robot()


    def turn_left(self, angle):
        """
            Turns the robot to the left by a given angle in degrees.
        """
        self.rotate(angle, "l")


    def turn_right(self, angle):
        """
            Turns the robot to the right by a given angle in degrees.
        """
        self.rotate(angle, "r")


    def drive_until_blocked(self):
        """
            Drives the robot forward until it is obstructed from in front.
        """
        self.rate.sleep()

        vel_msg = Twist()

        while not self.is_blocked_in_front():
            vel_msg.linear.x = 1
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        self.velocity_publisher.publish(vel_msg)


    def go_to_pose(self, x, y, heading):
        """
            Drives the robot to a specified pose.

            params:
                x, y: the position of the desired pose
                heading: the orientation of the desired pose in degrees
        """
        self.rate.sleep()

        vel_msg = Twist()

        # convert heading to radians, between -pi and pi
        if heading > 180:
            heading = heading - 360
        heading = math.radians(heading)

        # drive to desired position
        while self.euclidean_distance(x, y) > self.DISTANCE_TOLERANCE:
            steering_angle = self.steering_angle(x, y)

            if abs(steering_angle - self.theta) > self.ROTATION_TOLERANCE:
                vel_msg.linear.x = 0
                vel_msg.angular.z = abs(steering_angle - self.theta) * 0.5
            else:
                vel_msg.linear.x = self.LINEAR_VEL
                vel_msg.angular.z = 0

            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        # turn to desired orientation
        while abs(heading - self.theta) > self.ROTATION_TOLERANCE:
            vel_msg.linear.x = 0
            vel_msg.angular.z = abs(heading - self.theta) * 0.5
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        print(abs(heading - self.theta))

        self.stop_robot()


"""
if __name__ == "__main__":
    try:
        robot_controller = RobotController()

        while not rospy.is_shutdown():
            # robot_controller.turn_right(90)
            # robot_controller.go_to_pose(0, 0, 0)
            break

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
"""
