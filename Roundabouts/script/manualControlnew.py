import rospy
import tf
import math
import time
import datetime
import numpy as np
from numpy.linalg import inv,pinv
from geometry_msgs.msg import TransformStamped
import matplotlib.pyplot as plt
from std_msgs.msg import String
from socketIO_client import SocketIO, LoggingNamespace
from sensor_msgs.msg import Joy

#Control Tuning
turnScalar = 4
gasScalar = .5
#reverse = 0;
shiftLast = 0
steerAdjustLast = 0

def on_connect():
	print ('connected')
def cmdZumo(index,v,w):
#Send commands to zumos
#Add paper reference
	l = 0.9
	rv = v+0.5*l*w
	lv = v-0.5*l*w
 	cur_speed = (rv + lv)/2
	if(abs(cur_speed)>.5):
		eff = .5/abs(cur_speed)	
		rv = rv*eff
		lv = lv*eff
	velocities = str(index) + ':' + str(v) + ',' + str(w) + ';'# +str(reverse) +'&'
	print velocities
	socketIO.emit('getvelocity', velocities)
	
def callback(wheel): # string contains: i,xRef,yRef
    global turnScalar
    global gasScalar
    global shiftLast
    global steerAdjustLast

    steer = wheel.axes[0] * turnScalar
    gas = (wheel.axes[2]+1) * gasScalar
    #reverse = wheel.buttons[17]
    

    if(time.time()-shiftLast>.5):
	    if (wheel.buttons[4]==1):
		shiftLast = time.time()
		gasScalar = gasScalar +.1
		print("Gas Scalar increased to " + str(gasScalar))
    if(time.time()-shiftLast>.5 and gasScalar>.1):
	    if(wheel.buttons[5]==1):
		shiftLast = time.time()
		gasScalar = gasScalar -.1
		print("Gas Scalar decrease to " + str(gasScalar))
		#print("shiftLast: " + str(shiftLast) + " current time: " + str(time.time()))
    if(time.time()-steerAdjustLast>.5):
	    if (wheel.buttons[19]==1):
		steerAdjustLast = time.time()
		turnScalar = turnScalar +.5
		print("Turn Scalar increased to " + str(turnScalar))
	    if (wheel.buttons[20]==1 and turnScalar > .5):
	    	steerAdjustLast = time.time()
		turnScalar = turnScalar -.5
		print("Turn Scalar decreased to " + str(turnScalar))

    cmdZumo(11,gas,steer)

def main():
    global tfl
    #rospy.init_node('zumo_go') #getRefs')
    #tfl = tf.TransformListener()
    rospy.Subscriber("joy",Joy,callback)
    rospy.init_node('Joy2Turtle')
    print("Node created?")
    rospy.spin()

if __name__ == '__main__':
    global data
    global timeBefore
    global thetaBefore
    global timeStart
    socketIO = SocketIO('192.168.1.245', 5001, LoggingNamespace)
    socketIO.on('connect',on_connect)
    shiftLast = time.time()
    steerAdjustLast = time.time()
    main()

