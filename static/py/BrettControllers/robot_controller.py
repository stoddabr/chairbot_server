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
import math

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

    command()
        tells robot_entities to calculate paths and move robots to their goals

    getPositions( type: str )
        gets possible positions for a type (arrangemetn, formation, ect)

    saveNewPosition( name: str, positionType: str, category="temporary", author="Default"  )
        creates and saves a new position dataframe in the json file based on the current
        state of the robot entities

    setPositioning( positionType: str, name: str )
        recalls a positioning from the json file based on the position type and the name
        where applicable, it calculates the new goal coordinates based on that type and
        then sets that as the new goal for each entity

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
        self.MOVEMENT_LOCK = False # enabled for snap, formation for example
        self.LAST_SET_POSITON_INFO = False
        self.LAST_SET_POSITON_TYPE = False


    def updateRobotLocation(self, robotId, coords, toMoveOrNotToMove=True):
        """ Updates a robots saved location which can then be used to calculate
        robot movements

        Parameters
        ----------
        robotId : int
            robot fiducial id
        coords : tuple < int, int, float >
            (x,y,angle degrees) representing the location of the robot
        toMoveOrNotToMove : boolean  default True
            if true, will also move the robot
        """

        # if robot doesn't exist, initialize one
        if not self.robots.has_key( robotId ):
            self.robots[robotId] = RobotEntity( robotId )
        # update coords
        self.robots[robotId].updateCoords( coords )
        # check if movememnt lock and update goal if so
        if self.MOVEMENT_LOCK:
            self._updateGoalPositions()
        # move robot if no existential crisis
        if toMoveOrNotToMove:
            return self.robots[ robotId ].move()
        return 'stop', False # stop command has no goal

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


    def getPositions( self, posType ):
        """gets possible positions for a type (arrangement, formation, ect)


        reads from json file where the positioning information is stored

        Parameters
        ----------
        type : str
            The positioning type (arrangement, formation, ect)
        """

        return pos.getPositions(posType)

    def _getCurrentRobotPositions(self):
        """ gets the current locations for the robots"""

        info = {}
        for robotId, robotEntity in self.robots.items():
            # tuples and int64 are not json serializable
            # need to convert to array here
            tmp_coords = robotEntity.getCoords()
            if tmp_coords: #avoid None type
                info[robotId] = [int(x) for x in tmp_coords]
            else:
                info[robotId] = None

        return info

    def _getRelativeRobotPositions(self, masterId=5):
        """ gets the current locations for the robots relative to a master"""

        info = {}
        for robotId, robotEntity in self.robots.items():
          absCords = robotEntity.getCoords()
          if absCords:
              info[robotId] =  [int(x) for x in absCords]
        return {
            'coords': info,
            'originId': masterId,
        }


    def saveNewPosition( self, name, postype,  author="Default", category="temporary" ):
        """ creates and saves a new possible positions for a type (arrangement,
        formation, ect) based on current robot snapshot

        Parameters
        ----------
        name: str
            human-readable name for the position
        postype: str
            type of positioning. For example, "formation", "arrangement",
            "snap-orientation"
        """

        if (postype == 'arrangement'):
            info = self._getCurrentRobotPositions()
        elif (postype == 'formation'):
            info = self._getRelativeRobotPositions()
        else:
            raise Exception('Position type not recognized: '+postype)

        return pos.saveNewPosition(name, postype, info, author, category)


    def _setGoalsFromAbsolutePosition(self, positionData):
        """ sets goals from absolute positions

        inverse of _getCurrentRobotPositions(*)

        Parameters
        ----------
        positionData: dictionary
            dictionary with robotIds as keys and coordinate array as values
            where coordinate array is [x,y,angle]
        """

        for robotId, robotEntity in self.robots.items():
            if str(robotId) in positionData:
                coordsArray = positionData[str(robotId)]
                if coordsArray:
                    coords = tuple(coordsArray)
                    robotEntity.updateGoal(coords)


    def _rotate(self, origin, point, angle):
        """
        Rotate a point counterclockwise by a given angle around a given origin.

        The angle should be given in degrees.
        Converted from angles/counterclockwise from this stackoverflow
            https://stackoverflow.com/a/34374437/12334204

          Parameters
          ----------
          origin: array length 2
              x, y coordinates of the point to rotate around
          point: array length 2
              x, y coordinates of the point to be rotate
          angle: int
              type of positioning. For example, "formation", "arrangement",
              "snap-orientation"
        """

        ox, oy = origin
        px, py = point
        angleRad = math.radians(angle)
        qx = ox + math.cos(angleRad) * (px - ox) - math.sin(angleRad) * (py - oy)
        qy = oy + math.sin(angleRad) * (px - ox) + math.cos(angleRad) * (py - oy)
        return qx, qy

    def _calculateRelativePosition(self, origin, newOrigin, point):
        """ calculates relative position using previous and new origin

        for cases of rotation, secondary chairs match the primary instead of
        the entire formation rotating. Only calculated translation

        Parameters
        ----------
        origin: array
            [x, y, angle] coordinates of the point to rotate around
        newOrigin: array
            [x, y, angle] coordinates of the new origin coordinates
        point: array
            [x, y, angle] coordinates of the point to be rotate
        """
        # calculate origin translation and rotation
        print('new old', newOrigin, origin)
        dx = newOrigin[0] - origin[0]
        dy = newOrigin[1] - origin[1]
        dt = newOrigin[2] - origin[2]
        # calculate new point wrt origin translation
        x = point[0] + dx
        y = point[1] + dy
        t = newOrigin[2]
        return [x,y,t]


    def _calculateRelativePositionWithRotationLock(self, origin, newOrigin, point):
        """ DEPRECATED calculates relative position using previous and new origin
        while preserving the formation in the case of a rotation: if the primary robot
        turns, the formation turns.

        for cases of rotation, the entire formation rotates. Calculates then adds
        two linear transformations: rotation and translation.

        DEPRECATED because it causes goals to "jump" long distances for long "radii"
        between primary and secondary chairbots.

        Parameters
        ----------
        origin: array
            [x, y, angle] coordinates of the point to rotate around
        newOrigin: array
            [x, y, angle] coordinates of the new origin coordinates
        point: array
            [x, y, angle] coordinates of the point to be rotate
        """

        # calculate origin translation and rotation
        print('new old', newOrigin, origin)
        dx = newOrigin[0] - origin[0]
        dy = newOrigin[1] - origin[1]
        dt = newOrigin[2] - origin[2]
        # calculate new point wrt origin rotation
        xrot,yrot = self._rotate(origin[:2], point[:2], dt)
        # calculate new point wrt origin translation
        x = xrot + dx
        y = yrot + dy
        t = (point[2] + dt) % 360
        return [x,y,t]


    def _setGoalsFromRelativePosition(self, positionData):
        """ sets goals from absolute positions

        inverse of _getCurrentRobotPositions(*)

        Parameters
        ----------
        positionData: dictionary
            dictionary with robotIds as keys and coordinate array as values
            where coordinate array is [x,y,angle]
        """

        originId = positionData['originId']
        # avoid case where original origin robotId was not recorded
        if not positionData['coords'].has_key(str(originId)):
            errString = 'Brett: Origin id not recorded in coordinates: ' + str(originId)
            print('Error: '+errString)
            raise Exception(errString) # possibly comment out during production?
            return False
        # avoid case where origin robotId is not valid
        if not self.robots.has_key(originId):
            errString = 'Brett: Origin robot id not recorded in current entities: ' + originId
            print('Error: '+errString)
            raise Exception(errString) # possibly comment out during production?
            return False

        originCoords = positionData['coords'][ str(originId) ]
        print(originCoords)
        newOriginCoords = self.robots[originId].getCoords()
        for robotId, robotEntity in self.robots.items():
            if positionData['coords'].has_key(str(robotId)):
                recordedCoords = positionData['coords'][str(robotId)]
                newCoords = tuple(
                  self._calculateRelativePosition( originCoords, newOriginCoords, recordedCoords )
                )
                robotEntity.updateGoal(newCoords)
            else:
                print('robot not in coords', robotId)
        # raise Exception('see above')


    def _setSnapRoomGoals( self, positionData ):
        """ set angle based on snap-to-grid features

        Parameters
        ----------
        positionData: dictionary
            dictionary with robotIds as keys and coordinate array as values
            where coordinate array is [x,y,angle]
        """

        angle = positionData['dir']
        for robotId, robotEntity in self.robots.items():
            print(angle, robotId)
            goalArray = [False, False, angle]
            coords = tuple(goalArray)
            robotEntity.updateGoal(coords)


    def _setSnapObjectGoals( self, positionData ):
        """ set angle based on snap-to-grid features

        Parameters
        ----------
        positionData: dictionary
            dictionary with robotIds as keys and coordinate array as values
            where coordinate array is [x,y,angle]
        """

        angleDir = positionData['dir']
        objectId = int(positionData['id'])
        print(self.robots.keys())
        objectCoords = self.robots[objectId].getCoords()
        angle = ( objectCoords[2] + angleDir ) % 360
        for robotId, robotEntity in self.robots.items():
            print(angle, robotId)
            goalArray = [False, False, angle]
            coords = tuple(goalArray)
            robotEntity.updateGoal(coords)


    def _updateGoalPositions( self ):
        """ updates goal positions for all relevant robots based on the passed
        position information

        """

        position = self.LAST_SET_POSITON_INFO
        posType = self.LAST_SET_POSITON_TYPE
        print('Updating Goal Position based on: ',position)
        if not position: # avoid error if position is False
            return
        positionData = position['data']
        if posType == 'arrangement':
            # set goals as absolute positions
            self._setGoalsFromAbsolutePosition(positionData)
        elif posType == 'formation':
            # formations imply movement lock
            self.MOVEMENT_LOCK = True
            # set goals as calculated relative positions
            self._setGoalsFromRelativePosition(positionData)
        elif posType == 'snapObject': # TODO
            # formations imply movement lock
            self.MOVEMENT_LOCK = True
            # set angular snap goals
            self._setSnapObjectGoals(positionData)
        elif posType == 'snapRoom':
            # formations imply movement lock
            self.MOVEMENT_LOCK = True
            # set angular snap goals
            self._setSnapRoomGoals(positionData)
        else:
            errBase = 'Position type not equal to "formation" "arrangement" "snapRoom" or "snapObject": '
            raise Exception( errBase + posType )


    def setPositioning( self, posType, name ):
        """ recalls a positioning and updates goals for robotEntities

        Parameters
        ----------
        posType: str
            position type, ie "arrangement", "formation", "snapObject", "snapRoom"
        name: str
            human-readable name for the position
        """

        allPositions = pos.getPositions(posType, False)
        position = allPositions[name]
        self.LAST_SET_POSITON_TYPE = posType
        self.LAST_SET_POSITON_INFO = position
        self._updateGoalPositions()


    def stop( self ):
        """ sends stop command to all chairs and resets goals

        big red button
        """

        stopped = []
        self.LAST_SET_POSITON_TYPE = False
        self.LAST_SET_POSITON_INFO = False
        stopCommand = CommandClass( 'stop' )
        for robotId, robotEntity in self.robots.items():
            robotEntity.sendCommand( stopCommand )
            robotEntity.clearGoal()
            stopped.append( robotId )

        return {
          'stoppedRobotIds': stopped
        }
