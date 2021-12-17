#!/usr/bin/env python
import roslib; roslib.load_manifest("neato_node");
import rospy
import time

from what_is_my_name import what_is_my_name
from std_msgs.msg import Int8, String
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from neato_driver.neato_driver import Botvac


class PacketReplicatorNode:
    '''
    Replicates packets from the  requestMotion and requestStop packets
    '''

    def __init__(self):
        #each chair will have their own topics
        self.chairbot_number = what_is_my_name();
        #topic where we put the replicated packet
        self.chairMovement_topic_name = 'chairMovement' + self.chairbot_number
        #topic where we get the request motion packet
        self.requestMotion_topic_name = 'requestMotion' + self.chairbot_number
        #topic where we get the request stop packet
        self.requestStop_topic_name = 'requestStop' + self.chairbot_number
        self.chairMovementTopic = rospy.Publisher(self.chairMovement_topic_name, Twist, queue_size=30);



        #empty twist packet to replicate which we will fill with the right motion
        self.motion = None
        self.packet = Twist()

        #these are the motions which will be performed when a given CONSTANT is in the packet
        #this is what we actually replicate!
        self.BACKWARD = {
                      'linear': {'x': 150.0, 'y':0.0, 'z':0.0},
                      'angular': {'x': 0.0, 'y':0.0, 'z':0.0}
                   }

        self.FORWARD = {
                    'linear': {'x': -150.0, 'y':0.0, 'z':0.0},
                    'angular': {'x': 0.0, 'y':0.0, 'z':0.0}
                 }
        self.LEFT = {
                    'linear': {'x': 0.0, 'y':0.0, 'z':0.0},
                    'angular': {'x': 0.0, 'y':0.0, 'z':50.0}
        }
        self.RIGHT = {
                    'linear': {'x': 0.0, 'y':0.0, 'z':0.0},
                    'angular': {'x': 0.0, 'y':0.0, 'z':-50.0}
        }
        self.LEFT_SLOW = {
                    'linear': {'x': 0.0, 'y':0.0, 'z':0.0},
                    'angular': {'x': 0.0, 'y':0.0, 'z':10.0}
        }
        self.RIGHT_SLOW = {
                    'linear': {'x': 0.0, 'y':0.0, 'z':0.0},
                    'angular': {'x': 0.0, 'y':0.0, 'z':-10.0}
        }

        self.STOP_MOTION = {
                  'linear': {'x': 0.0, 'y':0.0, 'z':0.0},
                  'angular': {'x': 0.0, 'y':0.0, 'z':0.0}
                }
        

        #dict map of gestures which are special cases of motion over time
        #  a gesture consists of an array of tuples containing motion and time
        #  in seconds
        self.GESTURES = { 
        'SHAKE_SIDE': [(self.LEFT, 0.5), (self.RIGHT, 1)],
        'SHAKE_FRONT': [(self.BACKWARD, 1), (self.FORWARD, 1)], 
        'SPIN_180': [(self.LEFT, 1), (self.RIGHT, 2), (self.LEFT, 3)], 
        'SPIN_360': [(self.RIGHT, 0.5), (self.LEFT, 0.75), (self.RIGHT, 1)] 
        }
        self.ACTIVE_GESTURE = None  # when set, gestures will execute
        self.G_TIMER_START = 0.0
        self.G_TIMER_LEN = 0.0

        #dict mapping the constants to the actual motion dictionaries
        self.MOTIONS = { 'BACKWARD' : self.BACKWARD,
                    "FORWARD": self.FORWARD,
                    'LEFT': self.LEFT,
                    'RIGHT': self.RIGHT,
                    'STOP' : self.STOP_MOTION,
                    'RIGHT_SLOW': self.RIGHT_SLOW,
                    'LEFT_SLOW': self.LEFT_SLOW
                   }

        #this tracks whether we are told to stop or not
        self.STOP_FLAG = False

        #requestMotion topic which receives MOTION commands from the frontends
        rospy.Subscriber(self.requestMotion_topic_name, String, self.motion_callback, queue_size=10)
        #stopMotion topic which receives STOP commands from the frontend
        rospy.Subscriber(self.requestStop_topic_name, String, self.motion_callback, queue_size=10)
        #initialize a ros node
        rospy.init_node('packet_replicator_' + self.chairbot_number)


    def motion_callback(self, msg):
        '''
        takes the motion message request and sets flags OR motion variables based on it
        '''
        print "We got a msg", msg.data

        self.clear_gesture()  # on new command, overwrite current gesture

        msg = msg.data #just unrwap the command

        if msg == 'STOP': # we were given the STOP command
            rospy.loginfo("We got a STOP")
            self.STOP_FLAG = True
        else: # we were given a MOTION command
            self.STOP_FLAG = False

        if msg in self.GESTURES:
            print "gesture recognized"
            self.G_TIMER_START = time.time()
            self.ACTIVE_GESTURE = self.GESTURES[msg][:] # deep copy
        else:
            self.motion = self.MOTIONS[msg]  # where a message becomes a speed based on pre-defined values
        
        print("The motion is gonna be ", self.motion)


    def clear_gesture(self):
        """ cancels current gesture, useful for stop """

        self.ACTIVE_GESTURE = None
        self.motion = self.STOP_MOTION


    def gesture(self):
        """ gestures are special case motions which change over time """

        timer_start = self.G_TIMER_START
        t_now = time.time()
        time_elapsed = t_now - timer_start 
        if time_elapsed > self.G_TIMER_LEN:
            
            # if done executing final motion
            if self.ACTIVE_GESTURE == 'FINAL_GESTURE':
                self.clear_gesture()
                return

            # check next item in gesture list
            self.G_TIMER_START = t_now
            self.motion, self.G_TIMER_LEN = self.ACTIVE_GESTURE.pop(0)
              
            # ensure that final gesture gets executed  
            if len(self.ACTIVE_GESTURE) == 0:
                self.ACTIVE_GESTURE = 'FINAL_GESTURE'

    def spin(self):
        self.r = rospy.Rate(20) # WHY 20 ???
        while not rospy.is_shutdown():
            if self.ACTIVE_GESTURE is not None:
                self.gesture()

            if self.motion is None:
                #print "Waiting for motion"
                continue; #try again!

            if self.STOP_FLAG is True:
                # rospy.loginfo("Stopping")
                pass
            else:
                # rospy.loginfo("Moving")
                pass;

            #rospy.loginfo("Replicating the packet")
            #rospy.loginfo(self.motion)
            #populate the packet with the movememnt commands 
            #for that motion which were set by the motion_callback
            self.packet.linear.x = self.motion['linear']['x']
            self.packet.linear.y = self.motion['linear']['y']
            self.packet.linear.z = self.motion['linear']['z']
            self.packet.angular.x = self.motion['angular']['x']
            self.packet.angular.y = self.motion['angular']['y']
            self.packet.angular.z = self.motion['angular']['z']
            self.chairMovementTopic.publish(self.packet)
            self.r.sleep()

if __name__ == "__main__":
    robot = PacketReplicatorNode()
    robot.spin()

