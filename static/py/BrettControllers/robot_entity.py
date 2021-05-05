'''
Robot entity class

written by Brett Stoddard for the neato robot ChairBot

uses formatting based on this article
    https://realpython.com/documenting-python-code/

Changelogs:
- 6/4 Brett started class. Wrote basic public interfaces
- 6/20 Brett continued class. extended class to include ROS interface
'''
from math import sin, cos, atan2, sqrt, pi
import sys


import threading

from robot_command import CommandClass


# allow system to run on non-ros setups
try:
    import rospy
    from std_msgs.msg import String

    threading.Thread(target=lambda: rospy.init_node('robot_entity', disable_signals=True)).start()

    gen_move_task = lambda x: rospy.Publisher(
        ('/requestMotion0'+str(x)), String, queue_size=1)
    gen_stop_task = lambda x: rospy.Publisher(
        ('/requestStop0'+str(x)), String, queue_size=1)

except:
    class fakeRosPublisher:
        def __init__(self):
            pass
        def publish(self, *stuff):
            pass
        def __call__(self, *stuff):
            pass

    gen_move_task = fakeRosPublisher()
    gen_stop_task = fakeRosPublisher()

chair_ids = range(21)

pub_motion_arr = list(map(gen_move_task , chair_ids))
pub_stop_arr = list(map(gen_stop_task , chair_ids))


distTolerance = 10  # pixels FIXME experimentally determine
angleTolerance = 120 / 2  # degrees
angleToleranceTight = 30 / 2 # tolerance for fine/slow adjustments

class RobotEntity:
    """
    A class used to contain data about a specific robot such as it's goals and location

    Attributes
    ----------
    coords : tuple <int,int,int>
        x,y,angle current coordinates of the robot
    goal : tuple <int,int,int>
       x,y,angle coordinates of the robot's final goal location
    id : int
        the id assigned to this robot. Could be related to fiducial id

    Methods
    ----------
    updateCoords()
        update the internally saved coordinates for this robot

    getCoords()
        returns the most recent robot coordinates

    updateCoordinates()
        see updateCoords

    updateGoal( newGoal: tuple )
        update goal coordinates

    clearGoal()
        Resets a robot's saved goal

    calculateCommand( id: int ) -> CommandClass
        calculates and returns which command to send to the robot (eg turn, forward)

    move()
        triggers the execution of path planning software meticulously programmed
        by underpaid grad students to calculate the next before passing to generateCommand

    sendCommand( command: CommandClass )
        sends a command to the robot via ros
    """

    def __init__(self, robotId, coords=None, goal=None):
        """ Initializes a robot entity

        Parameters
        ----------
        id : int
            robot identifier
        command : string
            command string interperable by robot
        coords : tuple <int,int>, optional
            starting x,y coordinates of the robot
        goal : tuple < int, int > >
            tuple with x,y coordinates representing a robots goal location
        """

        self.robotId = robotId
        self.coords = coords
        self.goal = goal
        self.isStopped = False

    def updateCoords(self, newCoords):
        """ Updates robot's current location which can then be used to calculate
        robot movements

        Parameters
        ----------
        coords : tuple < int, int, float >
            x,y,angle coordinates
        """

        self.coords = newCoords

    def getCoords(self):
        """ returns the robots most recent coordinates """

        return self.coords

    def updateGoal(self, newGoal):
        """ Updates robot's goal location which can then be used to calculate
        robot movements

        Parameters
        ----------
        coords : tuple < int, int >
            x,y coordinates
        """

        print "Goal updated for "+str(self.robotId)+' '+str(newGoal)
        self.goal = newGoal

    def clearGoal(self):
        """ Resets a robot's goal

        """

        print 'clearing goal for robot '+str(self.robotId)
        self.goal = None

    def _calculateDistanceToGoal(self):
        """ Calculates and triggers a robot movement

        Returns int distance to goal in coords
        """

        [goalx, goaly, _] = self.goal
        [currx, curry, _] = self.coords  # current

        if goalx == False or goaly == False: # for angular goal
            return 0

        # eucledian distance
        xcube = (goalx-currx)**2
        ycube = (goaly-curry)**2
        return sqrt(xcube + ycube)

    def _calculateAngleToGoal(self):
        """ Calculates and triggers a robot movement

        Returns float angle in degrees [0,360]
        """

        [goalx, goaly, _] = self.goal
        [currx, curry, _] = self.coords  # current

        diffx = (goalx-currx)
        diffy = (goaly-curry)
        theta = atan2(diffy, diffx)
        return theta * 180 / pi + 180

    def _calculateAngularCommand(self):
        """ Calculates an angular command based on goal and theta angle

        Returns CommandClass instance
        """

        # check if angle within margin
        [_, _, goalAngle] = self.goal
        [_, _, currAngle] = self.coords
        # check if goalAngle is falsy
        if not goalAngle:
            return CommandClass('STOP')
        # calculate angle diff
        angleDiff = goalAngle - currAngle
        angleDiffAbs = abs(angleDiff)
        if(angleDiffAbs > 180):
             angleDiffAbs = 360 - angleDiffAbs
        # print('diff:,'+str(angleDiff)+', g: '+str(goalAngle)+', c:'+str(currAngle)+', |d|:'+str(angleDiffAbs))
        # use angleDiff to make next decision
        if angleDiffAbs < angleTolerance:
            if angleDiffAbs < angleToleranceTight:
                self.clearGoal()
                return CommandClass('STOP')
            if (angleDiff < -181 or 1 < angleDiff < 180):
                return CommandClass('RIGHT_SLOW')
            else:
                return CommandClass('LEFT_SLOW')
        if (angleDiff < -181 or 1 < angleDiff < 180):
            return CommandClass('RIGHT')
        else:
            return CommandClass('LEFT')

    def calculateCommand(self):
        """ Calculates the next best robot command to get towards the goal

        Returns CommandClass instance
        """

        # check if angular goal
        [goalx, goaly, _] = self.goal
        if goalx == False or goaly == False:
            return self._calculateAngularCommand()

        # check if distance within margin
        dist = self._calculateDistanceToGoal()
        if (dist < distTolerance):
            return self._calculateAngularCommand()

        # calculate angle to goal x,y coords
        goalAngle = self._calculateAngleToGoal()
        [_, _, currAngle] = self.coords
        angleDiff = goalAngle - currAngle
        angleDiffAbs = abs(angleDiff)
        if(angleDiffAbs > 180):
             angleDiffAbs = 360 - angleDiffAbs
        # print('diff:,'+str(angleDiff)+', g: '+str(goalAngle)+', c:'+str(currAngle)+', |d|:'+str(angleDiffAbs))
        # use angleDiff to make next decision
        if angleDiffAbs < angleTolerance:
            if angleDiffAbs < angleToleranceTight:
                return CommandClass('FORWARD')
            if (angleDiff < -181 or 1 < angleDiff < 180):
                return CommandClass('RIGHT_SLOW')
            else:
                return CommandClass('LEFT_SLOW')
        if (angleDiff < -181 or 1 < angleDiff < 180):
            return CommandClass('RIGHT')
        else:
            return CommandClass('LEFT')

    def move(self, newGoal=None):
        """ Calculates and triggers a robot movement

        Parameters
        ----------
        newGoal : tuple < int, int > optional
            new goal to move towards, will used saved goal if not defined

        Returns : tuple <str, goalCoords | False>
            Command string
            False if robot goal coords not set

        Raises
        ------
        SystemError
            If coords not found
        """

        # print "Moving robot "+str(self.robotId)+' to '+str(self.goal)
        if (newGoal != None):
            # TODO assert format before running
            self.goal = newGoal
        # abort if goals aren't defined
        if not self.goal:
            return 'No Goal', False
        if not self.coords:
            raise SystemError(
                'Location coordinates not defined for robot {}'
                .format(self.robotId)
            )
        # calculate and send command to neato
        command = self.calculateCommand()
        # required on init
        # self.sendCommand( CommandClass('STOP') )
        # print('trying to send command', command.generateCommand())
        return self.sendCommand(command), self.goal

    def sendCommand(self, command):
        """ Sends a command via ROS socket

        Parameters
        ----------
        command : CommandClass

        Raises
        ------
        SystemError
            If socket fails or if brett is lazy
        """

        id = self.robotId # TODO setup mapping from robotId to chairId
        message = command.generateCommand()
        if (message == 'STOP'):
            if not self.isStopped:
                print 'stopping robot '+str(self.robotId)
                pub_stop_arr[int(id)].publish( message )
                self.isStopped = True
        else:
            self.isStopped = False
            print 'sending to '+message+' '+str(id)
            pub_motion_arr[int(id)].publish( message )
        return message

#        raise SystemError('Command not implemented. Blame Brett Stoddard stoddardbrett@gmail.com')

        # TODO call rosbridge_websocket
        # @tutorial http://wiki.ros.org/roslibjs/Tutorials/ActionlibClient
        # method 1: try import ROSPY, init node, publish to topic
        #     http://wiki.ros.org/rospy/Tutorials
        #
        #     bash: rostopic echo /topicname  % this will display messages published to a topic
        #     python: import rospy
        #             # http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
        #
        # method 1.1: https://answers.ros.org/question/234418/easiest-way-to-implement-http-server-that-can-send-ros-messages/
        #    https://campus-rover.gitbook.io/lab-notebook/cr-package/web-application/flask-and-ros
        #    https://github.com/3SpheresRoboticsProject/flask_ask_ros/blob/master/src/skill_server.py
        #
        # method 2: reverse engineer roslibjs
