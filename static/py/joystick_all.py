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
  global Buttons
  global allB
  global last_1
  global last_2
  global Flag1
  global Flag2
  global Flag3
  global Flag4
  global Flag5

  Joy_commands =  ps
  Axess = ps.axes
  Buttons = ps.buttons
  now_1 = Axess[6]
  now_2 = Axess[7]
  allB = Buttons[4]

  if (allB):
    print 'Buttons[4] \n allB'
    Flag1 = True
    Flag2 = True
    Flag3 = True
    Flag4 = True
    Flag5 = True

  elif ((last_1==(-0.0) or last_1==1) and now_1==1):
    Flag1 = False
    Flag2 = False
    Flag3 = False
    Flag4 = True
    Flag5 = False

  elif ((last_1==(-0.0) or last_1==-1) and now_1==-1):
    Flag1 = False
    Flag2 = True
    Flag3 = False
    Flag4 = False
    Flag5 = False

  elif ((last_2==(-0.0) or last_2==-1) and now_2==-1):
    Flag1 = False
    Flag2 = False
    Flag3 = True
    Flag4 = False
    Flag5 = False

  elif ((last_2==(-0.0) or last_2==1) and now_2==1):
    Flag1 = True
    Flag2 = False
    Flag3 = False
    Flag4 = False
    Flag5 = False

  send_commands(Joy_commands)
  last_1=now_1
  last_2=now_2

def send_commands(ps):
  if (Flag5):
    pubjoy01.publish(ps)
    pubjoy02.publish(ps)
    pubjoy03.publish(ps)
    pubjoy04.publish(ps)
    print "Publishing to all"
  elif (Flag1):
    pubjoy01.publish(ps)
    print "Publishing to 1"
  elif Flag2:
    pubjoy02.publish(ps)
    print "Publishing to 2"
  elif (Flag3):
    pubjoy03.publish(ps)
    print "Publishing to 3"
  elif (Flag4):
    pubjoy04.publish(ps)
    print "Publishing to 4"

  print " Chair 1:",Flag1,"\n Chair 2:",Flag2,"\n Chair 3:",Flag3,"\n Chair 4:",Flag4,"\n All:",Flag5,"\n"


def listener():
  rospy.Subscriber("/joy", Joy , joy_handler)
  rospy.spin()

if __name__ == '__main__':
  Flag1 = False
  Flag2 = False
  Flag3 = False
  Flag4 = False
  Flag5 = True
  settings = termios.tcgetattr(sys.stdin)
  rospy.init_node('joystick', anonymous=True)
  pubjoy01 = rospy.Publisher("/joy01", Joy, queue_size=10)
  pubjoy02 = rospy.Publisher("/joy02", Joy, queue_size=10)
  pubjoy03 = rospy.Publisher("/joy03", Joy, queue_size=10)
  pubjoy04 = rospy.Publisher("/joy04", Joy, queue_size=10)
  
  try:
    print msg
    listener()
  except rospy.ROSInterruptException:
    pass