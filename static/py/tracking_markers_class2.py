'''
tracking_markers_class
version: 2

changed from version 1/0 to allow for an arbitrary
number of chairs. Instead of verbosely declaring every
chair as a variable in the class, now there is an array
of filenames saved in the self.filenames variable

TODO write a full description of the class
TODO understand the gemoetry used
'''

import cv2
import time
import math
import os
from datetime import datetime
import time

# import socket

# HOST = "127.0.0.1"
# PORT = "9600"

# variables that enable/disable features
WRITE_TO_FILE = False
STREAM_TO_ROBOT = True  # stream movement data to the robot


class TrackingCamera(object):
    def __init__(self, robotController):
        # robot controller class
        self.robotController = robotController

        # USB-Connected Camera
        self.cap = cv2.VideoCapture(1)  # 1 for usb camera, 0 for local

        # Fiducial Marker Dictionary
        self.dictionary = cv2.aruco.getPredefinedDictionary(
          cv2.aruco.DICT_4X4_50
          )

        # Constant relative file path to main.py
        self.filePath = ''
        print 'saving tracking information to directory: '+os.getcwd()

        # Initialize files using list comprehension
        self.numTrackers = 11 # default 8, higher fiducial numbers will be ignored
        self.filenames = [
            "{}chairbotTracking-CB0{}-{}.txt".format(
            self.filePath, str(i), time.strftime("%Y-%m-%d %H-%M-%S"))
            for i in xrange(self.numTrackers)
        ]

        if WRITE_TO_FILE:
            # test write to all initialized files
            columnLabelHeader = "x/ll[0] \t y/ll[1] \t degree \t time"
            for filepath in self.filenames:
                # Open and Write
                with open(filepath, 'w+') as f:
                    f.write(filepath)
                    f.write('\n')
                    f.write(columnLabelHeader)
                    f.write('\n')

        return;

    # Closing Process
    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()
        import sys
        sys.exit()
        return;

    # Start a
    # def socket_send(self, data):
    #     s = socket()
    #     s.bind((HOST, PORT))
    #     s.send("Hello ")
        # with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            # bind server to specific HOST and PORT
         #   s.bind((HOST, PORT))
            # send the payload data to the client
          #  s.send(data)
    # Returns an modified video image with tracking id markers
    def process(self):

        ret, frame = self.cap.read()
        gray = frame
        # detectMarkers returns: (corners, ids, rejectedImgPoints)
        corners, ids, _ = cv2.aruco.detectMarkers(gray,self.dictionary)

        # Checks all fiducials in ditionary
        if len(corners) > 0:
            # fids corners, findex fiducial index
            for (fids, index) in zip(corners, ids):
                for corner in fids: # pt => point number
                    try:
                        fid = int(index[0]) # defined fiducial id number

                        # exclude fiducial ids outside of expected range
                        if (fid >= 0 and fid <= self.numTrackers):

                            # ll contains (x, y) coordinate of the middle of fiducial
                            midcords = (corner[0] + corner[1] +corner[2] +corner[3]) \
                                /4 # average sum of 4 corners

                            # calculate angle from origin to fiducial center
                            # average of the top two fiducial corners
                            topcords = (corner[0] + corner[3]) / 2
                            # average of the bottome two fiducial corners
                            botcords = (corner[1] + corner[2]) / 2
                            # Difference between top and bottom
                            ydeltacords = topcords - botcords
                            # Tangent of the y and the x
                            theta = math.atan2(ydeltacords[1], ydeltacords[0])
                            # Changes theta from radians to positive degrees (0 to 360 rotating counter-clockwise)
                            degree = theta * (180 / math.pi) + 180

                            # draw circle on video stream to mark fiducial as processed
                            #circlesize = 15
                            #cv2.circle(gray,(midcords[0],midcords[1]), circlesize, (0,0,255), -1)

                            if WRITE_TO_FILE:
                                # Append data onto corresponding file
                                filename = self.filenames[int(index[0])]
                                with open(filename, 'a') as f:
                                    f.write(str(midcords[0])) # x
                                    f.write('\t')
                                    f.write(str(midcords[1])) # y
                                    f.write('\t')
                                    f.write(str(degree)) # angle
                                    f.write('\t')
                                    f.write(str(time.time())) # time
                                    f.write('\t\n')
                            if STREAM_TO_ROBOT:
                                # Stream movement commands to robot
                                # based on localization data
                                # print("Robot found", int(index[0]), midcords[0], midcords[1], degree )
                                command, goal = self.robotController.updateRobotLocation(
                                    int(index[0]), # fiducial id
                                    (midcords[0], midcords[1], degree), # x,y,angle, position tuple
                                )
                                # display sent commands and goal coordinates
                                font = cv2.FONT_HERSHEY_SIMPLEX
                                thickness = 2
                                if goal:
                                    color = (255,255,0)
                                    circlesize = 5
                                    fontScale = 0.5
                                    goalStr = 'Goal'+str(index[0])
                                    # print ('goal for ',index[0], goal)
                                    cv2.putText(gray,goalStr,(goal[0],goal[1]), font, fontScale, color, thickness, cv2.LINE_AA, False)
                                    cv2.circle(gray,(goal[0],goal[1]), circlesize, color, -1)
                                if command: # updateRobotLocation will return falsy if error
                                    # print('tracking markers command', command, index[0])
                                    color = (0,255,255)
                                    fontScale = 1
                                    cv2.putText(gray,command,(midcords[0],midcords[1]), font, fontScale, color, thickness, cv2.LINE_AA, False)


                    except IndexError:
                        pass

        # if no corners
        if len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(gray,corners,ids)
            pass

        # Turns into image
        ret, jpeg = cv2.imencode('.jpg', frame)
        return jpeg.tobytes()
