'''
Autononmy: Arrangements, Formations, and Grid (special case of formation)
written by Brett Stoddard for the neato robot ChairBot

uses formatting based on this article
https://realpython.com/documenting-python-code/


Assumes:
    Camera is at a constant position orthogonal to the floor

Changelogs:
- 7/16 Started module, wrote get and write functions

'''
import json
import time
from collections import defaultdict

#FILE_PATH = '/home/charisma/catkin_ws/src/chairbot_server/static/py/BrettControllers/saved/positions.json' FIXME
FILE_PATH = 'C:/Users/Brett/Documents/Robot/chairbot/chairbot_server/static/py/BrettControllers/saved/positions.json'


def getPositions(type, toJson=True):
    """ Reads and returns list of positions based on type

    Parameters
    ----------
    type : str
      position type, ie "arrangement", "formation", "snap"
    toJson : bool (optional)
      if the response should be a json string or dictionary
      default dictionary
    """

    with open(FILE_PATH) as f:
      data = json.load(f)

    # Output: {'name': 'Bob', 'languages': ['English', 'Fench']}
    positions = data[type]
    print(positions)

    if toJson:
      return json.dumps(positions)
    return positions

def saveNewPosition(name, type, positioningInfo):
    """ Saves new position to file

    Parameters
    ----------
    name: str
        human-readable name for the position
    type: str
        position type, ie "arrangement", "formation", "snap"
    positioningInfo : any
        data containing information that can be later recalled to snap
        the chairbots to desired positions.
        For example, the coordinates and angles of multiple chairbots
    """

    with open(FILE_PATH) as rf:
      data = json.load(rf)

    # prepare variables
    epoch_time = int(time.time())

    # add to dictionary
    if type in data:
        data[type][name] = {
            "epoch_time": epoch_time,
            "data":positioningInfo,
        }
    else:
        data[type] = {
            [name]: {
                "epoch_time": epoch_time,
                "data": positioningInfo,
            },
        }

    with open(FILE_PATH, 'w') as wf:
      json.dump(data, wf)

    return data
