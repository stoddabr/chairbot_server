'''
Robot location class

written by Brett Stoddard for the neato robot ChairBot

uses formatting based on this article
    https://realpython.com/documenting-python-code/

Changelogs:
- 6/20 Brett started class to create a standard way of sharing multiple locations
'''


class RobotLocations:
    """
    A class used to contain data about a specific robot such as it's goals and location

    Attributes
    ----------
    locations : list < tuple < tuple <int,int>, int > >
        x,y coordinates of the robot

    Methods
    ----------
    addItem

    """

    def __init__(self, id, coords=None, goal=None):
        """ Initializes a robot entity

        Paramet
