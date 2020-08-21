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
import position_helpers as pos

class RobotControllerClass:
    """
    A class used to track multi-robot information

    This is the main class used by the outside world

    Attributes
    ----------
    robots : dictionary < RobotEntity >
        dictionary of robots indexed by fiducial/robot id

    Public Methods
    -------
    updateRobotLocation( id: int , coords: tuple )
        updates a specific robot's location

    updateRobotGoal( robotId: int, coords: tuple )
        updates a robot's goal location coordinates and angles

    command( robotId: int )
        tells robot_entities to calculate paths and move robots to their goals

    getPositions( type: str )
        gets possible positions for a type (arrangemetn, formation, ect)

    createNewPositioning( info: obj )
        creates and saves a new possible positions for a type (arrangemetn,
        formation, ect) based on current robot snapshot

    setPositioning( positioning: obj )
        recalls a positioning

    stop()
        big red button to stop chair movement and reset goals
    """

    def __init__(self, fiducialIds=None):
        """
        Parameters
        ----------
        fiducialIds : array < int > optional
            array of fiducial/robot ids that may be active in the scene
        """

        if fiducialIds is not None:
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


    def getPositions( self, type ):
        """gets possible positions for a type (arrangement, formation, ect)


        reads from json file where the positioning information is stored

        Parameters
        ----------
        type : str
            The positioning type (arrangement, formation, ect)
        """

        return pos.getPositions(type)

    def _getCurrentRobotPositions(self):
        """ gets the current locations for the robots"""

        positionDict = {}
        for robotId, robotEntity in self.robots.items():
            # tuples and int64 are not json serializable
            # need to convert to array here
            tmp_coords = robotEntity.getCoords()
            if tmp_coords: #avoid None type
                positionDict[robotId] = [int(x) for x in tmp_coords]
            else:
                positionDict[robotId] = None

        return positionDict

    def _getCurrentRobotRelativePositions(self, originId=1):
        """ gets the current relative locations for the robots

        relative to entity with id originId
        """

        positionDict = {}
        origin_coords = self.robots[originId].getCoords()
        print(origin_coords)
        for robotId, robotEntity in self.robots.items():
            # tuples and int64 are not json serializable
            # need to convert to array here
            tmp_coords = robotEntity.getCoords()
            if tmp_coords: #avoid None type
                positionDict[robotId] = [
                    int(x)-int(x0) for x,x0 in zip(tmp_coords, origin_coords)
                ]
            else:
                positionDict[robotId] = None

        info = {
            'coords' : positionDict,
            'originId' : originId,
        }
        return info

    def saveNewPosition( self, name, type, category="temporary", author="Default" ):
        """ creates and saves a new possible positions for a type (arrangement,
        formation, ect) based on current robot snapshot

        Parameters
        ----------
        name: str
            human-readable name for the position
        type: str
            type of positioning. For example, "formation", "arrangement",
            "snap-orientation"
        """

        if (type == 'arrangement'):
            info = self._getCurrentRobotPositions()
        elif (type == 'formation'):
            info = self._getCurrentRobotRelativePositions()
        else:
            raise Exception('Position type not recognized: '+type)

        return pos.saveNewPosition(name, type, info, author, category)

    def setPositioning( self, type, name ):
        """ recalls a positioning and updates goals for robotEntities

        Parameters
        ----------
        positioning : dict
            name: str
                human-readable name for the position
            type: str
                position type, ie "arrangement", "formation", "snap"
            data : array <dict>
                data containing chairbot coordinates and angles
        """

        allPositions = pos.getPositions(type, False)
        print('get positions', type, name)
        position = allPositions[name]
        print position
        raise Exception('setPositioning not yet implemented')


    def stop( self ):
        """ sends stop command to all chairs and resets goals

        big red button
        """

        stopped = []

        stopCommand = CommandClass( 'stop' )
        for robotId, robotEntity in self.robots.items():
            robotEntity.sendCommand( stopCommand )
            robotEntity.clearGoal()
            stopped.append( robotId )

        return {
          'stoppedRobotIds': stopped
        }
