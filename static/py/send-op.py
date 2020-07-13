#!/usr/bin/env python
import roslib
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
'''
CHRIS's CODE
'''
import time
from std_msgs.msg import UInt16
from sensor_msgs.msg import Joy

#Global Variables
ang=90
inc=.5

key_pub = rospy.Publisher('servo', UInt16, queue_size=0)
#rospy.init_node("key_listen")
#rate = rospy.Rate(100)

msg = """
Reading from the keyboard  and Publishing to neato!
---------------------------
Moving around:
   q    w    e
   a    s    d
"""

moveBindings = {
		'w':(20,20,100),
		'a':(-20,20,100),
		's':(-20,-20,100),
		'd':(20,20,100),
		'q':(10,20,100),
		'e':(20,10,100),
		'x':(0,0,0),
}


speedBindings={
		'p':(),
}
def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def joy_handler(ps):
  print ps
  global ang
  global inc
  old_ang=ang
  if ps.axes[3]>0:
    ang+=inc
  elif ps.axes[3]<0:
    ang-=inc
  if ps.axes[3]==1:
    ang+=inc*2
  elif ps.axes[3]==-1:
    ang-=inc*2
  if ang>130:
      ang=130
  elif ang<45:
      ang=45
  if not ang==old_ang:
    print "pub!"
    key_pub.publish(ang)
  #time.sleep(.1)


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.ranges)
    
def listener():
    #rospy.Subscriber('neato_scan', LaserScan, callback)
    # spin() simply keeps python from exiting until this node is stopped

    r = rospy.Rate(5)
    while not rospy.is_shutdown():
    	key = getKey()
    	global cmd_vel

    	if key in moveBindings.keys():
    		print str(key)
    		cmd_vel.x = moveBindings[key][0]
    		cmd_vel.y = moveBindings[key][1]
    		cmd_vel.z = moveBindings[key][2]
    		pub.publish(cmd_vel)
    		#
    		#print "Going forward by : x - %s, y - %s, z - %s" % (cmd_vel.x,cmd_vel.y,cmd_vel.z)
    	elif key in speedBindings.keys():
    		sys.exit()
        
    	#print "Publishing : x - %s, y - %s, z - %s" % (cmd_vel.x,cmd_vel.y,cmd_vel.z)
    	r.sleep()

if __name__ == '__main__':
	settings = termios.tcgetattr(sys.stdin)
	rospy.init_node('sendop', anonymous=True)
	pub = rospy.Publisher("command", Vector3, queue_size=10)
	rospy.Subscriber("/joy", Joy , joy_handler)
	global cmd_vel
	cmd_vel = Vector3()
	try:
		print msg
		listener()
	except rospy.ROSInterruptException:
		pass