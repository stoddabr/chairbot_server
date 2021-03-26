'''
tracking_markers_class
version: 2

changed from version 1/0 to allow for an arbitrary
number of chairs. Instead of verbosely declaring every
chair as a variable in the class, now there is an array
of filenames saved in the self.filenames variable

TODO write a full description of the class
TODO understand the gemoetry used and compensate for height of markers
TODO calibrate camera for aeruco markers
'''

import cv2
import time
import math
import os
# from imutils.video import WebcamVideoStream
from datetime import datetime
import numpy as np
import time
from draw_arrow import drawArrow

# import socket

# HOST = "127.0.0.1"
# PORT = "9600"

robot_colors = {
 1: (0,204,0),
 2: (204,204,0),
 3: (0,102,204),
 4: (0,230,255),
}


# from https://www.codingforentrepreneurs.com/blog/open-cv-python-change-video-resolution-or-scale
def make_480p(cap):
    cap.set(3, 640)
    cap.set(4, 480)

# from https://stackoverflow.com/questions/60674501/how-to-make-black-background-in-cv2-puttext-with-python-opencv
def draw_text_bg(img, text,pos=(0, 0),font=cv2.FONT_HERSHEY_PLAIN,font_scale=3,text_color=(0, 255, 0),font_thickness=2,text_color_bg=(15, 15, 15)):
    x, y = pos
    x, y = int(x), int(y)
    text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)
    text_w, text_h = text_size
    print((x,y), (x + text_w, y + text_h), text_color_bg, -1)
    cv2.rectangle(img, (x,y), (int(x + text_w), int(y + text_h)), text_color_bg, -1)
    cv2.putText(img, text, (x, int(y + text_h + font_scale - 1)), font, font_scale, text_color, font_thickness)
    return text_size

class TrackingCamera(object):
    def __init__(self, robotController, WRITE_TO_FILE=False, STREAM_TO_ROBOT=True, OVERLAY_MOTION=False, OVERLAY_ID=False, OVERLAY_GOAL=True):
        # study config
        # variables that enable/disable features
        self.WRITE_TO_FILE = WRITE_TO_FILE
        self.STREAM_TO_ROBOT = STREAM_TO_ROBOT  # stream movement data to the robot
        self.OVERLAY_MOTION = False # overlay commands and goal in UI
        self.OVERLAY_ID = OVERLAY_ID
        self.OVERLAY_GOAL = OVERLAY_GOAL

        # robot controller class
        self.robotController = robotController

        # USB-Connected Camera
        self.cap = cv2.VideoCapture(1)  # 1 for usb camera, 0 for local

        # limit stream
        self.cap.set(cv2.CAP_PROP_FPS, 1); # internal buffer will now store only 3 frames

        # Fiducial Marker Dictionary
        self.dictionary = cv2.aruco.getPredefinedDictionary(
          cv2.aruco.DICT_4X4_50
          )

        # Constant relative file path to main.py
        self.filePath = ''
        print 'saving tracking information to directory: '+os.getcwd()

        # Initialize files using list comprehension
        self.numTrackers = 20 # default 8, higher fiducial numbers will be ignored
        self.filenames = [
            "{}chairbotTracking-CB0{}-{}.txt".format(
            self.filePath, str(i), time.strftime("%Y-%m-%d %H-%M-%S"))
            for i in xrange(self.numTrackers)
        ]

        if self.WRITE_TO_FILE:
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

        ret, framefull = self.cap.read() # full dimensions of the frame
        if not ret:
            ret, jpeg = cv2.imencode('.jpg', np.zeros((100,100,3), np.uint8)) # frame for og resolution
            return jpeg.tobytes()
        frame = framefull[:-150, 40:-100] # crop y,x  # crash here if framefull not defined
        # print 'frame size '+ str(frame.shape)
        gray = frame

        # overlay style defaults
        color = (255,255,0)
        circlesize = 5
        font = cv2.FONT_HERSHEY_SIMPLEX
        thickness = 2
        fontScale = 0.5

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


                            # draw arrow overlay on robots
                            if int(fid) in [0, 1, 2, 3, 4]:
                                if self.WRITE_TO_FILE:
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
                                if self.STREAM_TO_ROBOT:
                                    # Stream movement commands to robot
                                    # based on localization data
                                    # print("Robot found", int(index[0]), midcords[0], midcords[1], degree )
                                    command, goal = self.robotController.updateRobotLocation(
                                        int(index[0]), # fiducial id
                                        (midcords[0], midcords[1], degree), # x,y,angle, position tuple
                                    )
                                    # display sent commands and goal coordinates
                                    if goal:
                                        goalStr = 'Goal'+str(index[0])
                                        if self.OVERLAY_GOAL:
                                            # print ('goal for ',index[0], goal)
                                            # cv2.circle(gray,(int(goal[0]),int(goal[1])), circlesize, color, -1)
                                            draw_text_bg(gray,goalStr,(int(goal[0])+8,int(goal[1])-10), font, fontScale, robot_colors[fid], thickness)
                                            gray = drawArrow(gray, (int(goal[0]),int(goal[1])), int(goal[2]), robot_colors[fid], thickness=2, delta=7, offset=0)
                                    if command: # updateRobotLocation will return falsy if error
                                        # print('tracking markers command', command, index[0])
                                        if self.OVERLAY_MOTION:
                                            draw_text_bg(gray,command,(midcords[0],midcords[1]), font, 1, (0,255,255), thickness, cv2.LINE_AA, False)

                                    # chair text above everything else
                                    gray = drawArrow(gray, midcords, degree, robot_colors[fid], thickness = 4, delta=15, offset=15)
                                    draw_text_bg(gray,'Chair'+str(index[0]),(midcords[0]-20,midcords[1]-5), font, 0.5, robot_colors[fid], thickness)
                    except IndexError:
                        pass

        # if corners
        if len(corners) > 0:
            if self.OVERLAY_ID:
                cv2.aruco.drawDetectedMarkers(gray,corners,ids)
            pass

        # Turns into image
        scale_percent = 80 # percent of original size
        width = int(frame.shape[1] * scale_percent / 100)
        height = int(frame.shape[0] * scale_percent / 100)
        dim = (width, height)
        # resize image
        resized = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)

        ret, jpeg = cv2.imencode('.jpg', resized) # frame for og resolution
        return jpeg.tobytes()
