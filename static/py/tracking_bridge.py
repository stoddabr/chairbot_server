#!/usr/bin/env python

import roslib
import rospy

from geometry_msgs.msg import Twist

import sys
import select
import termios
import tty

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

import time
from std_msgs.msg import UInt16

msg = """
Reading from the GoPro Camera ~!! And Publishing to the ChairBots!
---------------------------
Place markers in front of the camera to start reading:
"""


def mark_handler():
    global marker_pose_1
    global marker_pose_2
    global marker_pose_3
    global marker_pose_4
    global marker_pose_5
    global marker_pose_6
    global marker_pose_7
    global marker_pose_8

    global Line1
    global Line2
    global Line3
    global Line4
    global Line5
    global Line6
    global Line7
    global Line8

    global host_busy
    global host_requested

    global Waypoint_Host
    global Waypoint_Path1
    global Waypoint_Path2
    global Waypoint_Path3
    global Waypoint_Table1
    global Waypoint_Table2

    Line1 = False
    Line2 = False
    Line3 = False
    Line4 = False
    Line5 = False
    Line6 = False
    Line7 = False
    Line8 = False

    host_busy = False
    host_requested = (0, 0)

    Waypoint_Host = (500, 500)
    Waypoint_Table1 = (0, 0)
    Waypoint_Table2 = (0, 0)
    Waypoint_Path1 = (0, 0)
    Waypoint_Path2 = (0, 0)
    Waypoint_Path3 = (0, 0)

    lastVal = [None] * 4

    print("reading")

    while True:
        print Line4
        filePath = '/home/charisma/catkin_ws/src/central_server/static/chairbot/py/'
        filename1 = open(filePath + "CB01.txt", 'r')
        lastLine1 = filename1.readlines()[-1].strip('\n')
        if ".txt" not in lastLine1:
            vals = map(float, lastLine1.strip().split('\t'))
            marker_pose_1 = vals[0:3]
            if(marker_pose_1 != lastVal):
                lastVal = marker_pose_1
                print marker_pose_1
            Line1 = True
            pass
        filename1.close()

        filename2 = open(filePath + "CB02.txt", 'r')
        lastLine2 = filename2.readlines()[-1].strip('\n')
        if ".txt" not in lastLine2:
            vals = map(float, lastLine2.strip().split('\t'))
            marker_pose_2 = vals[0:3]
            Line2 = True
            pass
        filename2.close()

        filename3 = open(filePath + "CB03.txt", 'r')
        lastLine3 = filename3.readlines()[-1].strip('\n')
        if ".txt" not in lastLine3:
            vals = map(float, lastLine3.strip().split('\t'))
            marker_pose_3 = vals[0:3]
            Line3 = True
            pass
        filename3.close()

        filename4 = open(filePath + "CB04.txt", 'r')
        lastLine4 = filename4.readlines()[-1].strip('\n')
        if ".txt" not in lastLine4:
            vals = map(float, lastLine4.strip().split('\t'))
            marker_pose_4 = vals[0:3]
            Line4 = True
            print Line4
            pass
        filename4.close()

        filename5 = open(filePath + "CB05.txt", 'r')
        lastLine5 = filename5.readlines()[-1].strip('\n')
        if ".txt" not in lastLine5:
            vals = map(float, lastLine5.strip().split('\t'))
            marker_pose_5 = vals[0:3]
            Line5 = True
            pass
        filename5.close()

        filename6 = open(filePath + "CB06.txt", 'r')
        lastLine6 = filename6.readlines()[-1].strip('\n')
        if ".txt" not in lastLine6:
            vals = map(float, lastLine6.strip().split('\t'))
            marker_pose_6 = vals[0:3]
            Line6 = True
            pass
        filename6.close()

        filename7 = open(filePath + "CB07.txt", 'r')
        lastLine7 = filename7.readlines()[-1].strip('\n')
        if ".txt" not in lastLine7:
            vals = map(float, lastLine7.strip().split('\t'))
            marker_pose_7 = vals[0:3]
            Line7 = True
            pass
        filename7.close()

        filename8 = open(filePath + "CB08.txt", 'r')
        lastLine8 = filename8.readlines()[-1].strip('\n')
        if ".txt" not in lastLine8:
            vals = map(float, lastLine8.strip().split('\t'))
            marker_pose_8 = vals[0:3]
            Line8 = True
            pass
        filename8.close()

        send_commands()
    pass


def get_current_waiter_waypoint(current_pose):

    pass


def get_current_host_waypoint(current_pose):
    global host_busy
    global host_requested

    global Waypoint_Host
    global Waypoint_Path1
    global Waypoint_Path2
    global Waypoint_Path3
    global Waypoint_Table1
    global Waypoint_Table2

    if(not Line5 and not Line6 and not Line7 and not Line8):
        return None
    else:
        if(not host_busy):
            if(Line5):
                if((abs(marker_pose_5[0] - Waypoint_Host[0]) < 10) and (abs(marker_pose_5[1] - Waypoint_Host[1]) < 10)):
                    host_busy = True
                    host_requested = Waypoint_Host
                    return Waypoint_Host

            elif(Line6):
                if((abs(marker_pose_6[0] - Waypoint_Host[0]) < 10) and (abs(marker_pose_6[1] - Waypoint_Host[1]) < 10)):
                    host_busy = True
                    host_requested = Waypoint_Host
                    return Waypoint_Host

            elif(Line6):
                if((abs(marker_pose_7[0] - Waypoint_Host[0]) < 10) and (abs(marker_pose_7[1] - Waypoint_Host[1]) < 10)):
                    host_busy = True
                    host_requested = Waypoint_Host
                    return Waypoint_Host

            elif(Line8):
                if((abs(marker_pose_8[0] - Waypoint_Host[0]) < 10) and (abs(marker_pose_8[1] - Waypoint_Host[1]) < 10)):
                    host_busy = True
                    host_requested = Waypoint_Host
                    return Waypoint_Host
        else:
            if((abs(current_pose[0] - host_requested[0]) < 10) and (abs(current_pose[1] - host_requested[1]) < 10) and host_requested == Waypoint_Host):
                host_busy = False
                greetRoutine()

            if((abs(current_pose[0] - host_requested[0]) < 10) and (abs(current_pose[1] - host_requested[1]) < 10) and host_requested == Waypoint_Path1):
                host_requested = Waypoint_Path2

            if((abs(current_pose[0] - host_requested[0]) < 10) and (abs(current_pose[1] - host_requested[1]) < 10) and host_requested == Waypoint_Path2):
                host_requested = Waypoint_Table1

            if((abs(current_pose[0] - host_requested[0]) < 10) and (abs(current_pose[1] - host_requested[1]) < 10) and host_requested == Waypoint_Table1 or host_requested == Waypoint_Table2):
                host_busy = False
                tableRoutine()


def greetRoutine():
    # Spin
    # Shine Light
    host_busy = True
    host_requested = Waypoint_Path1


def tableRoutine():
    # Do Something
    host_busy = True
    host_requested = Waypoint_Host


def go_to_waypoint(current_pose, waypoint, msg_pub):
    # current_pos (x, y)
    # waypoint (x, y)
    # we want to compute a vector pointing from current_pos to waypoint
    vector_x = waypoint[0] - current_pose[0]
    vector_y = waypoint[1] - current_pose[1]
    angle_to_waypoint = math.atan2(vector_y, vector_x)
    diff = angle_to_waypoint - current_pose[2]
    steering_msg = Twist()
    if abs(diff) > 10:
        steering_msg.angular.z = sign(diff)*50
    else:
        sterring_msg.linear.x = 50
    msg_pub.publish(steering_msg)


def send_commands():
    rate = rospy.Rate(10)  # 10hz
    # last line whatever is the coord

    if (Line3):
        waypoint = get_current_waiter_waypoint(marker_pose_3)
        if waypoint is not None:
            go_to_waypoint(marker_pose_3, waypoint, pubmark03)
        else:
            pubmark03.publish(Twist())

    if (Line4):
        waypoint = get_current_host_waypoint(marker_pose_4)
        if waypoint is not None:
            go_to_waypoint(marker_pose_4, waypoint, pubmark04)
        else:
            pubmark04.publish(Twist())

    print Line1, Line2, Line3, Line4, Line5, Line6, Line7, Line8


# def listener():
#     rospy.Subscriber("/mark", Twist , mark_handler)
#     rospy.spin()


if __name__ == '__main__':

    rospy.init_node('mark', anonymous=True)
    pubmark01 = rospy.Publisher("/mark01", Twist, queue_size=10)
    pubmark02 = rospy.Publisher("/mark02", Twist, queue_size=10)
    pubmark03 = rospy.Publisher("/mark03", Twist, queue_size=10)
    pubmark04 = rospy.Publisher("/mark04", Twist, queue_size=10)
    pubmark05 = rospy.Publisher("/mark05", Twist, queue_size=10)
    pubmark06 = rospy.Publisher("/mark06", Twist, queue_size=10)
    pubmark07 = rospy.Publisher("/mark07", Twist, queue_size=10)
    pubmark08 = rospy.Publisher("/mark08", Twist, queue_size=10)

    try:
        print msg
        mark_handler()
    except rospy.ROSInterruptException:
        pass
