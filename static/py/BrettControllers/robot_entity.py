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

# import rospy FIXME

from robot_command import CommandClass


class RobotEntity:
    """
    A class used to contain data about a specific robot such as it's goals and location

    Attributes
    ----------
    coords : tuple <int,int>
        x,y coordinates of the robot
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
    calculateCommand( id: int ) -> Command
        calculates which command to send to the robot (eg turn, forward)
    sendCommand( command: Command)
        sends a command to the robot
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

        self.goal = newGoal

    def _calculateDistanceToGoal(self):
        """ Calculates and triggers a robot movement

        Returns int distance to goal in coords
        """
        [goalx, goaly] = self.goal
        [currx, curry, _] = self.coords  # current

        # eucledian distance
        xcube = (goalx-currx)**2
        ycube = (goaly-curry)**2
        return sqrt(xcube + ycube)

    def _calculateAngleToGoal(self):
        """ Calculates and triggers a robot movement

        Returns float angle in degrees [0,360]
        """
        [goalx, goaly] = self.goal
        [currx, curry, _] = self.coords  # current

        diffx = (goalx-currx)
        diffy = (goaly-curry)
        theta = atan2(diffy, diffx)
        return theta * 180 / pi + 180

    def generateCommand(self):
        """ Calculates and triggers a robot movement

        Returns int distance to goal in coords
        """

        distTolerance = 10  # pixels FIXME experimentally determine
        angleTolerance = 45 / 2  # degrees

        # check if distance within margin
        dist = _calculateDistanceToGoal()
        if (dist < distTolerance):
            return CommandClass('Nothing', self.robotId)

        # check if angle within margin
        goalAngle = _calculateAngleToGoal()
        [_, _, currAngle] = self.coords
        if goalAngle + angleTolerance > currAngle \
                and goalAngle - angleTolerance < currAngle:
            return CommandClass('Left', self.robotId)

        # turn so angle is within margin
        if goalAngle < currAngle:
            return CommandClass('Right', self.robotId)
        else:
            return CommandClass('Left', self.robotId)

    def move(self, newGoal=None):
        """ Calculates and triggers a robot movement

        Parameters
        ----------
        newGoal : tuple < int, int > optional
            new goal to move towards, will used saved goal if not defined

        Returns : boolean
            True if robot motion was sent
            False if robot motion was not sent

        Raises
        ------
        SystemError
            If coords not found
        """

        if (newGoal != None):
            # TODO assert format before running
            self.goal = newGoal

        # abort if goals aren't defined
        if not self.goal:
            return False

        if not self.coords:
            raise SystemError(
                'Location coordinates not defined for robot {}'
                .format(self.robotId)
            )

        # calculate and send command to neato
        command = self.generateCommand(goalCoords)
        if command.isNothing():
            return False

        self.sendCommand(command)

        return True

    def sendCommand(self, command):
        """ Sends a command over ROS socket

        Parameters
        ----------
        command : CommandClass

        Raises
        ------
        SystemError
            If socket fails
        """

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
