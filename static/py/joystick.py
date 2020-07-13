#!/usr/bin/env python
import roslib
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

import time
from std_msgs.msg import UInt16
from sensor_msgs.msg import Joy

msg = """
Reading from the Dualshock4 controller ~!! And Publishing to neato!
---------------------------
Move around:
"""

global last_1
global last_2
last_1=0
last_2=0
def joy_handler(ps):
  global Joy_commands
  global Axess
  global last_1
  global last_2
  global Flag1
  global Flag2
  global Flag3
  global Flag4

  Joy_commands =  ps
  Axess = ps.axes
  #print Axess[6]
  now_1 = Axess[6]
  now_2 = Axess[7]
  if ((last_1==(-0.0) or last_1==1) and now_1==1):
  	Flag4 = True
  	Flag1 = False
  	Flag2 = False
  	Flag3 = False
  	#print "Publishing to 4"

  elif ((last_1==(-0.0) or last_1==-1) and now_1==-1):
  	Flag2 = True
  	Flag1 = False
  	Flag4 = False
  	Flag3 = False
  	#print "Publishing to 2"

  elif ((last_2==(-0.0) or last_2==-1) and now_2==-1):
  	Flag3 = True
  	Flag1 = False
  	Flag2 = False
  	Flag4 = False
  	#print "Publishing to 3"

  elif ((last_2==(-0.0) or last_2==1) and now_2==1):
  	Flag1 = True
  	Flag4 = False
  	Flag2 = False
  	Flag3 = False
  	#print "Publishing to 1"

  send_commands(Joy_commands)
  last_1=now_1
  last_2=now_2

def send_commands(ps):
  if (Flag1):
  	pubjoy01.publish(ps)
  	print "Publishing to 1"
  elif Flag2:
  	pubjoy02.publish(ps)
  	print "Publishing to 2"
  elif (Flag3):
  	pubjoy03.publish(ps)
  	print "Publishing to 3"
  elif (Flag4):
  	pubjoy01.publish(ps)
  	pubjoy02.publish(ps)
  	pubjoy03.publish(ps)
  	print "Publishing to 4"

  print Flag1,Flag2,Flag3,Flag4


def listener():
	rospy.Subscriber("/joy", Joy , joy_handler)
	rospy.spin()

if __name__ == '__main__':

	Flag1 = False
	Flag2 = False
	Flag3 = False
	Flag4 = False
	settings = termios.tcgetattr(sys.stdin)
	rospy.init_node('joystick', anonymous=True)
	pubjoy01 = rospy.Publisher("/joy01", Joy, queue_size=10)
	pubjoy02 = rospy.Publisher("/joy02", Joy, queue_size=10)
	pubjoy03 = rospy.Publisher("/joy03", Joy, queue_size=10)
	#pubjoy04 = rospy.Publisher("/joy04", Joy, queue_size=10)
	try:
		print msg
		listener()
	except rospy.ROSInterruptException:
		pass
