#!/usr/bin/env python3.8

import math
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Twist
from laser_geometry.laser_geometry import LaserProjection


class Planner:
    def __init__(self):
        # Creates a node with name 'Planner' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node("planner", anonymous=True)

        # Publisher which will publish to the topic '/cmd_vel'.
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.point_cloud_publisher = rospy.Publisher(
            "/point_cloud", PointCloud2, queue_size=10
        )

        # A subscriber to the topic '/odom'. self.update_pose is called
        # when a message of type Pose is received.
        rospy.Subscriber("/odom", Odometry, self.update_pose)

        # Laser data
        rospy.Subscriber("/base_scan", LaserScan, self.update_scan)

        self.projector = LaserProjection()

        self.scan = None
        self.point_cloud = None
        self.pose = None
        self.twist = None
        self.rate = rospy.Rate(10)

        self.PIXEL_TO_METER = 2
        self.DISTANCE_TOLERANCE = 0.1

    def update_pose(self, data):
        self.pose = data.pose.pose
        self.twist = data.twist.twist

    def update_scan(self, data):
        self.scan = data
        self.point_cloud = self.projector.projectLaser(data)

    def euclidean_distance(self, x, y):
        return math.sqrt(
            pow((x - self.pose.position.x), 2) + pow((y - self.pose.position.y), 2)
        )

    def is_blocked_in_right(self, distance):
        for i in range(166):
            if self.scan.ranges[i] < distance / self.PIXEL_TO_METER:
                return True
        return False

    def is_blocked_in_front(self, distance):
        for i in range(166, 334):
            if self.scan.ranges[i] < distance / self.PIXEL_TO_METER:
                return True
        return False

    def is_blocked_in_left(self, distance):
        for i in range(334, len(self.scan.ranges)):
            if self.scan.ranges[i] < distance / self.PIXEL_TO_METER:
                return True
        return False

    def drive_forward(self, distance):
        pass

    def turn_left(self, angle):
        pass

    def turn_right(self, angle):
        pass

    def drive_until_blocked(self, distance):
        pass

    def go_to_pose(self, x, y, heading):
        pass


if __name__ == "__main__":
    try:
        planner = Planner()
        rospy.sleep(1)

        while not rospy.is_shutdown():
            print(
                "---------------------------------DATA FROM LASER---------------------------------"
            )
            print("Left: " + str(planner.is_blocked_in_left(1)))
            print("Front: " + str(planner.is_blocked_in_front(1)))
            print("Right: " + str(planner.is_blocked_in_right(1)))
            planner.rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
