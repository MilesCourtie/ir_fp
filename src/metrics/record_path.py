#!/usr/bin/env python
"""
    ir_fp/src/metrics/record_path.py

    description: 

    usage: rosrun ir_fp record_path.py

    NOTE: This program assumes that the orientations of the received poses
    have x- and y-components of 0 (i.e. the robot is parallel to the floor).
    Assuming this, the heading of the robot can be calculated from just the
    w-component.
"""

#==================== imports ====================#

import rospy
import datetime
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from ir_fp.srv import RecordPathControl, RecordPathControlResponse


#==================== constants ====================#

FILENAME_FORMAT = "recorded_path_{timestamp:s}.csv"
BATCH_SIZE      = 200 # save to file every N measurements


#==================== globals ====================#

enabled = False

filename = ""

empty = True   # tracks whether initial pose has been stored

prev_pos_x = 0 # most recent x position
prev_pos_y = 0 # most recent y position
prev_rot_w = 0 # most recent w component of orientation

record_num = 0 # record index within current batch

# double buffer of records
records = ([0]*BATCH_SIZE, [0]*BATCH_SIZE)
record_buf = 0 # index of current buffer


#==================== functions ====================#

def save_batch(num_records):
    global filename
    global records
    global record_buf

    batch = records[record_buf]
    record_buf = 1 - record_buf
    with open(filename, "a") as f:
        f.writelines(batch[:num_records])
    print(str(num_records) + " records saved to " + filename)

# handler for when a base pose is received
def handle_base_pose(data):
    global enabled
    global empty
    global prev_pos_x
    global prev_pos_y
    global prev_rot_w
    global record_num
    global records
    global record_buf

    if not enabled:
        return

    pos_x = data.pose.pose.position.x
    pos_y = data.pose.pose.position.y
    rot_w = data.pose.pose.orientation.w

    if empty \
            or pos_x != prev_pos_x \
            or pos_y != prev_pos_y \
            or rot_w != prev_rot_w:
        prev_pos_x = pos_x
        prev_pos_y = pos_y
        prev_rot_w = rot_w
        empty = False

        # save record to buffer
        record = f"{pos_x},{pos_y},{rot_w}"
        print("record " + str(record_num) + ": " + record)
        records[record_buf][record_num] = record + "\n"
        record_num = record_num + 1
        if record_num == BATCH_SIZE:
            save_batch(BATCH_SIZE)
            record_num = 0

# handler for when a control signal is received
def handle_control(control):
    global enabled
    global filename
    global record_num

    print("control received: " + str(control))
    if control.enable == enabled:
        return RecordPathControlResponse(False) # no state change
    else:
        if control.enable:
            # start recording
            filename = FILENAME_FORMAT.format(timestamp=
                    datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
                    )
            record_num = 0
            record_buf = 0
            enabled = True
            empty = True
            print("recording started, filename: " + filename)
        else:
            # stop recording
            enabled = False
            save_batch(record_num)
            print("recording stopped")
        return RecordPathControlResponse(True) # state change


#==================== main ====================#

def record_path():
    rospy.init_node("record_path", anonymous=True)
    rospy.Subscriber("base_pose_ground_truth", Odometry, handle_base_pose)
    srv = rospy.Service("record_path_control", RecordPathControl, handle_control)
    rospy.spin()

if __name__ == "__main__":
    record_path()
