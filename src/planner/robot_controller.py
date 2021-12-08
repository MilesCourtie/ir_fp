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


# wrapper around rospy.ROSInterruptException
RobotInterruptException = rospy.ROSInterruptException


class RobotController:

    def __init__(self, node_name):

        # ROS node initialisation
        rospy.init_node(node_name, anonymous=True)

        # publisher for sending velocity commands
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # subscriber for receiving pose estimates
        rospy.Subscriber("/base_pose_ground_truth", Odometry, self.__update_pose)

        # subscriber for receiving laser scan data
        rospy.Subscriber("/base_scan", LaserScan, self.__update_scan)

        # the most recent laser scan data
        self.scan = None

        # the most recent pose estimate
        self.x = 0
        self.y = 0
        self.theta = 0

        # desired rate at which to publish velocity commands
        self.rate = rospy.Rate(10) # Hz

        # tolerances for the robot's estimated pose being equal to a desired pose
        self.DISTANCE_TOLERANCE = 0.001 # metres
        self.ROTATION_TOLERANCE = 0.001 # radians

        # forwards/backwards speed
        self.LINEAR_VEL = 1 # arbitrary units

        
    def __update_pose(self, data):
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


    def __update_scan(self, data):
        """
            Callback for when new laser scan data is received.
            Updates self.scan from the received data.
        """
        self.scan = data


    def __euclidean_distance(self, x, y):
        """
            Returns the Euclidean distance between the most recent pose
            estimate and a given XY coordinate.
        """
        return math.sqrt(pow((x - self.x), 2) + pow((y - self.y), 2))


    def __steering_angle(self, x, y):
        """
            Returns the angle in radians, between -Pi and Pi, between the
            positive X-axis and the vector from the most recent pose
            estimate to a given XY coordinate.
        """
        return math.atan2(y - self.y, x - self.x)


    def __stop_robot(self):
        """
            Publishes a velocity command to stop the robot.
        """
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)


    def is_shutdown(self):
        """
            Wrapper around rospy.is_shutdown().
        """
        return rospy.is_shutdown()

    def wait(self):
        """
            Wrapper around rospy.spin().
        """
        return rospy.spin()


    def is_blocked_right(self):
        """
            Returns true iff the robot is obstructed on the right-hand side.
        """
        self.rate.sleep()
        DETECTION_DISTANCE = 1.1
        for i in range(0, len(self.scan.ranges) // 10):
            if self.scan.ranges[i] < DETECTION_DISTANCE:
                return True


    def is_blocked_front(self):
        """
            Returns true iff the robot is obstructed from in front.
        """
        self.rate.sleep()
        mid = len(self.scan.ranges) // 2
        width = len(self.scan.ranges) // 19
        DETECTION_DISTANCE = 0.85
        for i in range(mid - width, mid + width):
            if self.scan.ranges[i] < DETECTION_DISTANCE:
                return True
        return False


    def is_blocked_left(self):
        """
            Returns true iff the robot is obstructed on the left-hand side.
        """
        self.rate.sleep()
        DETECTION_DISTANCE = 1.1
        for i in range(-len(self.scan.ranges) // 10, -1):
            if self.scan.ranges[i] < DETECTION_DISTANCE:
                return True
        return False


    def drive_forward(self, distance):
        """
            Publishes velocity commands to drive the robot forwards until
            it has moved a specified distance, then stops the robot.
        """
        self.rate.sleep()

        x = self.x
        y = self.y

        vel_msg = Twist()
        last_distance = 0
        while abs(distance) > self.DISTANCE_TOLERANCE:
            
            vel_msg.linear.x = self.LINEAR_VEL * max(0.01,min(abs(distance),1)) * (distance / abs(distance))
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
            last_distance = distance
            self.rate.sleep()
            
            walked_distance = self.__euclidean_distance(x, y)
            if distance <-self.DISTANCE_TOLERANCE:
                distance += walked_distance
            else:
                distance -= walked_distance

            x = self.x
            y = self.y

        self.__stop_robot()


    def __rotate(self, angle, direction):
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

        vel_msg = Twist()

        # calculate desired orientation in radians
        angle = (rotation_direction * math.radians(angle)) + self.theta

        # restrict desired orientation to be between -pi and pi
        if abs(math.degrees(angle)) > 180:
            angle = rotation_direction * math.radians(abs(math.degrees(angle)) - 360)

        while abs(angle - self.theta) > self.ROTATION_TOLERANCE:
            vel_msg.angular.z = abs(angle - self.theta) * rotation_direction
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        self.__stop_robot()


    def turn_left(self, angle):
        """
            Turns the robot to the left by a given angle in degrees.
        """
        self.__rotate(angle, "l")


    def turn_right(self, angle):
        """
            Turns the robot to the right by a given angle in degrees.
        """
        self.__rotate(angle, "r")


    def drive_until_blocked(self):
        """
            Drives the robot forward until it is obstructed from in front.
        """
        self.rate.sleep()

        vel_msg = Twist()

        while not self.is_blocked_front():
            vel_msg.linear.x = self.LINEAR_VEL
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        self.__stop_robot()


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
        while self.__euclidean_distance(x, y) > self.DISTANCE_TOLERANCE:
            steering_angle = self.__steering_angle(x, y)

            if abs(steering_angle - self.theta) > self.ROTATION_TOLERANCE:
                vel_msg.linear.x = 0
                vel_msg.angular.z = abs(steering_angle - self.theta)
            else:
                vel_msg.linear.x = self.LINEAR_VEL
                vel_msg.angular.z = 0

            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        # turn to desired orientation
        while abs(heading - self.theta) > self.ROTATION_TOLERANCE:
            vel_msg.linear.x = 0
            vel_msg.angular.z = abs(heading - self.theta)
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        self.__stop_robot()
