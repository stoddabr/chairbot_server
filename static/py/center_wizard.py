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

global DIST
global SPEED
Axess = (-0.0, -0.0, 1.0, -0.0, -0.0, 1.0, -0.0, -0.0)
Butt = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
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
  global DIST
  global SPEED
  global Axess
  global Butt
  global xlast
  global ylast
  global AageR
  global AageL
  global loop

  Lft_d = Axess[1]
  Rgh_t = Axess[3]

  AageL = Axess[2]
  AageR = Axess[5]
  #touch = Butt[13]
  SPEED -= ((AageR-1)*1)
  SPEED += ((AageL-1)*1)
  if (SPEED<0):
    SPEED=0
  elif (SPEED>330):
    SPEED=330

  SPEED = int(SPEED)

  ll = (Lft_d*DIST)
  rr = (Rgh_t*DIST)
  if (rr>=0):
    x = (-ll - rr)
    y = (-ll + rr)
  else:
    x = (-ll - rr)
    y = (-ll + rr) 
  #print AageR
  x=int(x)
  y=int(y)
  print (x,y,SPEED)

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

  now_1 = Axess[6]
  now_2 = Axess[7]
  global command_xy
  command_xy.x = x
  command_xy.y = y
  command_xy.z = SPEED

  last_1=now_1
  last_2=now_2

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
  send_commands(command_xy)
  rospy.spin()

if __name__ == '__main__':

  Flag1 = False
  Flag2 = False
  Flag3 = False
  Flag4 = False
  DIST=20
  global SPEED
  global xlast
  global ylast
  global AageR
  global AageL
  global loop
  global command_xy
  command_xy = Vector3()
  command_xy.x=0
  command_xy.y=0
  command_xy.z=0
  loop =0
  AageR=1
  AageL=1
  SPEED=100
  xlast=0
  ylast=0

  settings = termios.tcgetattr(sys.stdin)
  rospy.init_node('center_wizard', anonymous=True)
  pubjoy01 = rospy.Publisher("/joy01", Vector3, queue_size=10)
  pubjoy02 = rospy.Publisher("/joy02", Vector3, queue_size=10)
  pubjoy03 = rospy.Publisher("/joy03", Vector3, queue_size=10)
  #pubjoy04 = rospy.Publisher("/joy04", Joy, queue_size=10)

  try:
    print msg
    listener()
  except rospy.ROSInterruptException:
    pass
