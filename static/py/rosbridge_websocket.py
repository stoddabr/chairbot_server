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
from std_msgs.msg import Int8
from std_msgs.msg import UInt16
from sensor_msgs.msg import Joy

msg = """
**Connection successful**
Now you are reading from the UI and Publishing to the neato!
---------------------------
use the UI to move the chair around:
"""


def listener():
    rospy.spin()


if __name__ == '__main__':

    rospy.init_node('switch', anonymous=True)
    # TODO comment out code below
    pubon01 = rospy.Publisher("/cbon01", Int8, queue_size=10)
    pubon02 = rospy.Publisher("/cbon02", Int8, queue_size=10)
    pubon03 = rospy.Publisher("/cbon03", Int8, queue_size=10)
    pubon04 = rospy.Publisher("/cbon04", Int8, queue_size=10)
    # pubon05 = rospy.Publisher("/cbon05", Int8, queue_size=10)
    try:
        print(msg)
        listener()
    except rospy.ROSInterruptException:
        pass
