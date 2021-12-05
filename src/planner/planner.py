#!/usr/bin/env python3.8

import math
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Twist


class Planner:
    def __init__(self):
        # Creates a node with name 'Planner' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('planner', anonymous=True)

        # Publisher which will publish to the topic '/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # A subscriber to the topic '/odom'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.update_pose)

        # Laser data
        self.scan_subscriber = rospy.Subscriber('/base_scan', LaserScan, self.update_scan)

        self.scan = LaserScan()
        self.pose = Pose()
        self.twist = Twist()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        self.pose = data.pose.pose
        self.twist = data.twist.twist

    def update_scan(self, data):
        self.scan = data

    def euclidean_distance(self, x, y):
        return math.sqrt(
            pow((x - self.pose.position.x), 2) +
            pow((y - self.pose.position.y), 2)
        )

    def is_blocked_in_left(self, distance):
        pass

    def is_blocked_in_right(self, distance):
        pass

    def is_blocked_in_front(self, distance):
        pass

    def drive_forward(self, distance):
        vel_msg = Twist()
            
        vel_msg.linear.x = 1
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
            
        while(distance > 0):
	    self.velocity_publisher.publish(vel_msg)
	    distance -= 1
	    
	vel_msg.linear.x = 0

    def turn_left(self, angle):
        vel_msg = Twist()
            
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        
        vel_msg.angular.x = -1
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
            
        while(angle > 0):
	    self.velocity_publisher.publish(vel_msg)
	    angle -= 1
	    
	vel_msg.angular.x = 0
        
    def turn_right(self, angle):
        vel_msg = Twist()
            
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        
        vel_msg.angular.x = 1
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
            
        while(angle > 0):
	    self.velocity_publisher.publish(vel_msg)
	    angle -= 1
	    
	vel_msg.angular.x = 0

    def drive_until_blocked(self, distance):
        pass

    def go_to_pose(self, x, y, heading):
        pass
