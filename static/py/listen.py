#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.ranges)
    
def listener():
    rospy.init_node('listen', anonymous=True)
    rospy.Subscriber('neato_scan', LaserScan, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
