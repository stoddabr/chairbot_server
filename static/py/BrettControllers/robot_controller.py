'''
Robot controllers

written by Brett Stoddard for the neato robot ChairBot

uses formatting based on this article
https://realpython.com/documenting-python-code/

uses calulations from this related project
https://github.com/charisma-lab/neato_localization/blob/master/scripts/localizing_tracked_markers.py

TODO change docstrings to match epydoc format for easy documentation generation
http://epydoc.sourceforge.net/epytext.html


Assumes: 
    Camera is at a constant position orthogonal to the floor

Changelogs:
- 6/4 started class. Wrote basic public interfaces
- 6/20 continued class. extended class to include ROS interface, removed origin

'''
from robot_command import CommandClass
from robot_entity import RobotEntity


class RobotController:
    """
    A class used to track multi-robot information

    This is the main class used by the outside world

    Attributes
    ----------
    robots : dictionary < RobotEntity >
        dictionary of robots indexed by fiducial/robot id
    originId : int
        fiducial id of the origin reference used
    originLocation : tuple < int, int >
        x,y coordinates of the origin fiducial
    socket : ???
        socket used to send commands

    Public Methods
    -------
    updateRobotLocation( id: int , coords: tuple )
        updates a specific robot's location
    command #TODO  
    """

    def __init__(self, fiducialIds, originId, originCoords=None):
        """
        Parameters
        ----------
        robotIds : array < int >
            array of fiducial/robot ids that may be active in the scene
        originId : int
            fiducial id of the origin point
        originCoords : tuple <int, int>, optional
            x, y location
        """

        self.originId = originId
        self.originCoords = originCoords
        self.robots = {x: RobotEntity(x) for x in fiducialIds}

    def updateRobotLocation(self, robotId, coords):
        """ Updates a robots saved location which can then be used to calculate
        robot movements

        Parameters
        ----------
        robotId : int
            robot fiducial id
        coords : tuple < int, int, float >
            (x,y,angle degrees) representing the location of the robot
        """

        # if robot doesn't exist, initialize one
        if not self.robots.has_key(robotId):
            self.robots[robotId] = RobotEntity(robotId)

        # update coords
        self.robots[robotId].updateCoords(coords)

    def updateRobotGoal(self, robotId, coords):
        """ Updates a robots saved goal which can then be used to calculate
        robot movements

        Parameters
        ----------
        robotId : int
            robot fiducial id
        coords : tuple < int, int >
            (x,y) representing the location of the goal
        """
        # if robot doesn't exist, initialize one
        if not self.robots.has_key(robotId):
            self.robots[robotId] = RobotEntity(robotId)

        self.robots[robotId].updateGoal(coords)

    def command(self, robotId, newRobotCoords=None):  # TODO finish writing
        """Calculates a robots best move in order to move toward it's goal
        based on it's last saved coordinates

        this method can be called right after setting the robots location
        and that command can be sent in a loop until the next time a location is
        passed

        Parameters
        ----------
        robotId : int
            The sound the animal makes (default is None)
        newRobotCoords : tuple <int,int,int>, optional
            x,y,angle coordinates for this robot's new location. 
            If not provided, then last saved location of the robot will be used

        Raises
        ------
        SystemError
            If robotId not found
        """

        if not self.robots.has_key(robotId):
            raise SystemError(
                'Command sent to a robotId not initialized: {}'
                .format(robotId)
            )

        # update robot coord if defined
        if newRobotCoords != None:
            self.updateRobotGoal(robotId, newRobotCoords)

        # prepare and send move command to robot
        self.robots[robotId].move()
