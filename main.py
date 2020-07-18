#!/usr/bin/env python

# SOURCE: https://blog.miguelgrinberg.com/post/video-streaming-with-flask

import sys
from flask import Flask, render_template, Response, request, jsonify
import os
# import rospy
import threading
import html
# from std_msgs.msg import String
from static.py.tracking_markers_class2 import TrackingCamera
from static.py.BrettControllers.robot_controller import RobotControllerClass

app = Flask(__name__, template_folder='templates')

# ROS node, publisher, and parameter.
# The node is started in a separate thread to avoid conflicts with Flask.
# The parameter *disable_signals* must be set if node is not initialized
# in the main thread.
# tutorial

# threading.Thread(target=lambda: rospy.init_node('test_node', disable_signals=True)).start()
# setup topics related to each chairbot
f = '''
chair_ids = range(4)
gen_move_task = lambda x: rospy.Publisher(
    ('/requestMotion0'+str(x)), String, queue_size=1)
gen_stop_task = lambda x: rospy.Publisher(
    ('/requestStop0'+str(x)), String, queue_size=1)
pub_motion_arr = list(map(gen_move_task , chair_ids))
pub_stop_arr = list(map(gen_stop_task , chair_ids))
'''

fiducialIds = [1,2,3,4]
RobotController = RobotControllerClass()

@app.route('/')
def index():
    return render_template('index.html')

def gen(camera):
    while True:
        frame = camera.process()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')


@app.route('/video_feed')
def video_feed():
    return Response(gen(TrackingCamera(RobotController)),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


# get/set formations and arrangements
@app.route('/autonomy/<type>', methods = ['GET', 'PATCH', 'POST'])
def arrange():
    if request.method == 'GET':
        return RobotController.getPositions(type)

    elif request.method == 'PATCH':
        httpBody = request.get_json(force=True)
        name = httpBody['name']
        return RobotController.saveNewPosition(name, type)

    elif request.method == 'PATCH':
        httpBody = request.get_json(force=True)
        return RobotController.setPosition(httpBody) # TODO implement

    else:
        raise Exception('Route "/autonomy/<type>", method not accepted')


# directly control the robot
@app.route('/move/<direction>/<id>', methods = ['GET','POST'])
def send_movement_command(direction):
    if any(direction in d for d in ['forward','backward','left','right', 'stop']):
        # new ROSLIB.Message({data: motion})
        if (direction == 'stop'):
            pub_stop_arr[id].publish( direction.upper() )
            return '<h2>Stop Command Published</h2>'
        else:
            pub_motion_arr[id].publish( direction.upper() )
            return '<h2>Direction Command Published</h2>'
    else:
        mgs = 'Direction not recognized'
        return '<h2>Direction not recognized: unable to publish</h2>'
'''

@app.route('/get_formations')
def add_numbers():
    a = request.args.get('a', 0, type=int)
    b = request.args.get('b', 0, type=int)
    return jsonify(result=a + b)

if __name__ == '__main__':
	app.run(host='0.0.0.0', debug=False)
