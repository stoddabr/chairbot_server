#!/usr/bin/env python

# SOURCE: https://blog.miguelgrinberg.com/post/video-streaming-with-flask

import sys
from flask import Flask, render_template, Response
import os
import rospy
import threading
import html
from std_msgs.msg import String
from static.py.tracking_markers_class2 import TrackingCamera

app = Flask(__name__, template_folder='templates')

# ROS node, publisher, and parameter.
# The node is started in a separate thread to avoid conflicts with Flask.
# The parameter *disable_signals* must be set if node is not initialized
# in the main thread.
# tutorial

threading.Thread(target=lambda: rospy.init_node('test_node', disable_signals=True)).start()
# setup topics related to each chairbot
chair_ids = range(4)
gen_move_task = lambda x: rospy.Publisher(
    ('/requestMotion0'+str(x)), String, queue_size=1)
gen_stop_task = lambda x: rospy.Publisher(
    ('/requestStop0'+str(x)), String, queue_size=1)
pub_motion_arr = list(map(gen_move_task , chair_ids))
pub_stop_arr = list(map(gen_stop_task , chair_ids))

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
    return Response(gen(TrackingCamera()),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

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


if __name__ == '__main__':
	app.run(host='0.0.0.0', debug=False)
