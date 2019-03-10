#!/usr/bin/env python
#author Nikhil Kanamarla 
#imports for ros, python-related, and alexa
import os
import rospy
import threading
from flask import Flask
from flask_ask import Ask, question, statement
from std_msgs.msg import String
import sys
import os
sys.path.insert(0,'/home/themainframe/catkin_ws/src/line_following/script')


#creating the flask asp
app = Flask(__name__)
ask = Ask(app, "/")

# ROS node, publisher, and parameter.
# The node is started in a separate thread to avoid conflicts with Flask.
# The parameter *disable_signals* must be set if node is not initialized
# in the main thread.

threading.Thread(target=lambda: rospy.init_node('test_node', disable_signals=True)).start()
pub = rospy.Publisher('test_pub', String, queue_size=1)
NGROK = rospy.get_param('/ngrok', None)

#initial code used when skill is launched
@ask.launch
def launch():
    welcome_sentence = 'Hello, this is a test skill. Please state a command.'
    return question(welcome_sentence)

#code used when command for cars is given
@ask.intent('carIntent')
def carIntent(coffeeShop):
    #nkMergeSim.watchout()
    #launches external file
    os.system('python /home/themainframe/catkin_ws/src/line_following/script/nkMergeSim.py')
    pub.publish(coffeeShop)
    return statement('I have published the following name to a ROS topic:(coffeeShop)')

#return message to ngrok 
@ask.session_ended
def session_ended():
    return "{}", 200

#main method mainly used for ngrok
if __name__ == '__main__':
    if NGROK:
        print 'NGROK mode'
        app.run(host=os.environ['ROS_IP'], port=5000)
    else:
        print 'Manual tunneling mode'
        dirpath = os.path.dirname(__file__)
        cert_file = os.path.join(dirpath, '../config/ssl_keys/certificate.pem')
        pkey_file = os.path.join(dirpath, '../config/ssl_keys/private-key.pem')
        app.run(host=os.environ['ROS_IP'], port=5000,
                ssl_context=(cert_file, pkey_file))
