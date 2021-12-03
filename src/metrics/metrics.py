#!/usr/bin/python
"""
    ir_fp/src/metrics/metrics.py
    TODO description

    NOTE: This program assumes that the orientations of the received poses
    have x- and y-components of 0 (i.e. the robot is horizontal). Assuming
    this, the heading of the robot can be extracted from the w-component
    alone.
"""

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from ir_fp.srv import EnableMetrics, EnableMetricsResponse

# store previous pose to check whether robot has moved
prev_pos_x = 0 # x position
prev_pos_y = 0 # y position
prev_rot_w = 0 # w component of orientation

# handler for when a base pose is received
def handle_base_pose(data):
    global prev_pos_x
    global prev_pos_y
    global prev_rot_w
    
    pos_x = data.pose.pose.position.x
    pos_y = data.pose.pose.position.y
    rot_w = data.pose.pose.orientation.w

    if pos_x != prev_pos_x or pos_y != prev_pos_y or rot_w != prev_rot_w:
        print(f"{pos_x}\t{pos_y}\t{rot_w}")
        prev_pos_x = pos_x
        prev_pos_y = pos_y
        prev_rot_w = rot_w

# handler for when an enable/disable signal is received
# TODO implement enable/disable feature
def handle_enable(enable):
    print("enable: " + str(enable))
    return EnableMetricsResponse(enable)
    
# main body of the node
def metrics():
    rospy.init_node("metrics", anonymous=True)
    rospy.Subscriber("base_pose_ground_truth", Odometry, handle_base_pose)
    srv = rospy.Service("enable_metrics", EnableMetrics, handle_enable)
    rospy.spin()

if __name__ == "__main__":
    metrics()
