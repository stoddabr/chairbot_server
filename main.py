#!/usr/bin/env python

# SOURCE: https://blog.miguelgrinberg.com/post/video-streaming-with-flask

import sys
from flask import Flask, render_template, Response, request, jsonify
import os
import rospy
from std_msgs.msg import String, Int8
import threading
import html
from static.py.tracking_markers import TrackingCamera
from static.py.BrettControllers.robot_controller import RobotControllerClass


# study config files
SHOW_AR_OVERLAY = True  # overlay goal and command data
USER_ID = 'TEST'  # will change logging data


print('starting flask')
app = Flask(__name__, template_folder='templates')

# ROS node, publisher, and parameter.
# The node is started in a separate thread to avoid conflicts with Flask.
# The parameter *disable_signals* must be set if node is not initialized
# in the main thread.
# tutorial
print('starting ros in new thread')
# threading.Thread(target=lambda: rospy.init_node('test_node', disable_signals=True)).start()

# setup topics related to each chairbot
chair_ids = range(5)
gen_move_task = lambda x: rospy.Publisher(
    ('/requestMotion0'+str(x)), String, queue_size=1
)
gen_stop_task = lambda x: rospy.Publisher(
    ('/requestStop0'+str(x)), String, queue_size=1
)
gen_toggle = lambda x : rospy.Publisher(
    ('/cbon0'+str(x)), Int8, queue_size=1
)

pub_motion_arr = list(map(gen_move_task , chair_ids))
pub_stop_arr = list(map(gen_stop_task , chair_ids))
pub_toggle_arr = list(map(gen_toggle , chair_ids))

RobotController = RobotControllerClass(chair_ids)

@app.route('/')
def index():
    return render_template('index.html', user_tracking_id=USER_ID)

def gen(camera):
    while True:
        frame = camera.process()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')


@app.route('/video_feed')
def video_feed():
    return Response(gen(TrackingCamera(RobotController, DEBUG_OVERLAY=SHOW_AR_OVERLAY)),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


# get/set formations and arrangements
@app.route('/autonomy/<type>', methods = ['GET', 'POST', 'DELETE'])
def arrange(type):
    if request.method == 'GET':
        return RobotController.getPositions(type)

    elif request.method == 'POST':
        httpBody = request.get_json(force=True)
        name = httpBody['name'].encode('ascii','replace')
        RobotController.setPositioning(type, name)
        return 'Position set success'

    elif request.method == 'DELETE':
        RobotController.stop()
        return 'Robots stopped'

    else:
        raise Exception('Route "/autonomy/<type>", method not accepted')


# get/set formations and arrangements
@app.route('/new/autonomy/<type>', methods = ['POST'])
def record_position(type):
    if request.method == 'POST':
        httpBody = request.get_json(force=True)
        name = httpBody['name'].encode('ascii','replace')
        author = httpBody['author'].encode('ascii','replace')
        category = httpBody['category'].encode('ascii','replace')
        RobotController.saveNewPosition(name, type, author, category)
        return 'Position save success'
    else:
        raise Exception('Route "/autonomy/<type>", method not accepted')

# directly set goal position TODO implement in UI
@app.route('/goal/autonomy', methods = ['POST'])
def set_goal():
    if request.method == 'POST':
        httpBody = request.get_json(force=True)
        robotId = httpBody['id']
        x_goal = httpBody['x']
        y_goal = httpBody['y']
        angle_goal = httpBody['angle']
        coords = (x_goal, y_goal, angle_goal)
        RobotController.updateRobotGoal(robotId, coords)
        return 'Goal set success'
    else:
        raise Exception('Route "/goal/autonomy", method not accepted')

@app.route('/toggle/<command>/<id>', methods = ['POST'])
def toggle_chair(command, id):
    if request.method == 'POST':
        if command == 'enable':
            pub_toggle_arr[int(id)].publish(1)
            return 'enable published'
        elif command == 'disable':
            pub_toggle_arr[int(id)].publish(0)
            return 'disable published'
        else:
            print("toggle command not recognized.", command)
            return "toggle command not recognized: should be enable or disable"
    else:
        raise Exception('Route "/autonomy/<type>", method not accepted')

# directly control the robot
@app.route('/move/<direction>/<id>', methods = ['GET','POST'])
def send_movement_command(direction, id):
    if any(direction in d for d in ['FORWARD','BACKWARD','LEFT','RIGHT', 'STOP']):
        # new ROSLIB.Message({data: motion})
        if (direction == 'STOP'):
            print 'stopping robot '+str(id)
            pub_stop_arr[int(id)].publish( 'STOP' )
            return '<h2>Stop Command Published</h2>'
        else:
            pub_motion_arr[int(id)].publish( direction.upper() )
            return '<h2>Direction Command Published</h2>'
    else:
        mgs = 'Direction not recognized'
        return '<h2>Direction not recognized: unable to publish</h2>'



if __name__ == '__main__':
    app.run(threaded=True, host='0.0.0.0', debug=False)
