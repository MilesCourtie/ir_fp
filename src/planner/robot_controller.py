#!/usr/bin/env python3

import math
import rospy
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class RobotController:
    def __init__(self):
        # Creates a node with name 'Planner' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node("robot_controller", anonymous=True)

        # Publisher which will publish to the topic '/cmd_vel'.
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # A subscriber to the topic '/odom'. self.update_pose is called
        # when a message of type Pose is received.
        rospy.Subscriber("/odom", Odometry, self.update_pose)

        # Laser data
        rospy.Subscriber("/base_scan", LaserScan, self.update_scan)

        self.scan = None
        self.x = 0
        self.y = 0
        self.theta = 0
        self.rate = rospy.Rate(10)

        self.PIXEL_TO_CM = 2
        self.DISTANCE_TOLERANCE = 0.1
        # rotation tolerance in radians
        self.ROTATION_TOLERANCE = 0.001
        self.LINEAR_VEL = 1
        self.ANGULAR_VEL = math.radians(30)
        self.IS_BLOCKED_READING = 0.7

    def update_pose(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

        quaternion = data.pose.pose.orientation
        (_, _, self.theta) = euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        )

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def update_scan(self, data):
        self.scan = data

    def is_blocked_in_right(self):
        self.rate.sleep()

        for i in range(166):
            if self.scan.ranges[i] < self.IS_BLOCKED_READING:
                return True

        return False

    def is_blocked_in_front(self):
        self.rate.sleep()

        for i in range(166, 334):
            if self.scan.ranges[i] < self.IS_BLOCKED_READING:
                return True

        return False

    def is_blocked_in_left(self):
        self.rate.sleep()

        for i in range(334, len(self.scan.ranges)):
            if self.scan.ranges[i] < self.IS_BLOCKED_READING:
                return True

        return False

    def euclidean_distance(self, x, y):
        return math.sqrt(pow((x - self.x), 2) + pow((y - self.y), 2))

    def steering_angle(self, x, y):
        return math.atan2(y - self.y, x - self.x)

    def stop_robot(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def drive_forward(self, distance):
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
        self.rate.sleep()

        if angle == 0:
            return

        if direction == "l":
            rotation_direction = 1
        elif direction == "r":
            rotation_direction = -1

        print("current theta: " + str(math.degrees(self.theta)))

        vel_msg = Twist()

        angle = (rotation_direction * math.radians(angle)) + self.theta

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
        self.rotate(angle, "l")

    def turn_right(self, angle):
        self.rotate(angle, "r")

    def drive_until_blocked(self):
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
        self.rate.sleep()

        vel_msg = Twist()

        if heading > 180:
            heading = heading - 360

        heading = math.radians(heading)

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

        while abs(heading - self.theta) > self.ROTATION_TOLERANCE:
            vel_msg.linear.x = 0
            vel_msg.angular.z = abs(heading - self.theta) * 0.5
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        print(abs(heading - self.theta))

        self.stop_robot()


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
