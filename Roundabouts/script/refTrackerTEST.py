#!/usr/bin/env python
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

###################         For Post Processing        ###################

def dataCollection(data,index,x,y,cur_speed): ##
    global timeStart
    timestamp = str(time.time()-timeStart)
    data += timestamp + ',' + str(index)+ ',' + str(x)+','+str(y) + ',' + str(cur_speed) +';'
    return data

def saveFile(data): ##
    currentDate = time.strftime('%m_%d_%y__%H_%M_%S')
    newData = open("/home/themainframe/catkin_ws/src/line_following/data/data_%s.csv" %currentDate,"w")
    num=data.count(";")
    splitData = data.split(";")
    print "num: " + str(num)
    for m in range(num):
        newData.write(splitData[m])
        newData.write("\n")

########################################################################

def on_connect():
	print ('connected')

def getViconPos(index):
    global tfl
    try:
        zumoName =["zumoTest1","zumoTest2","zumoTest3","zumoTest4","zumoTest5","zumoTest6","zumoTest7","zumoTest8","zumoTest9","zumoTest10","zumoTest11","zumoTest12","zumoTest13","zumoTest14","zumoTest15","zumoTest16","zumoTest17","zumoTest18","zumoTest19","zumoTest20","zumoTest21","zumoTest22","zumoTest23","zumoTest24","zumoTest25","zumoTest26","zumoTest27","zumoTest28","zumoTest29","zumoTest30","zumoTest31","zumoTest32","zumoTest33","zumoTest34","zumoTest35"]
        frameName = zumoName[index-1]
        t = tfl.getLatestCommonTime("/static_corner_1", "/vicon/"+frameName+"/"+frameName)
        position1, quaternion1 = tfl.lookupTransform("/static_corner_1", "/vicon/"+frameName+"/"+frameName, t)
        t = tfl.getLatestCommonTime("/static_corner_2", "/vicon/"+frameName+"/"+frameName)
        position2, quaternion2 = tfl.lookupTransform("/static_corner_2", "/vicon/"+frameName+"/"+frameName, t)
        t = tfl.getLatestCommonTime("/static_corner_3", "/vicon/"+frameName+"/"+frameName)
        position3, quaternion3 = tfl.lookupTransform("/static_corner_3", "/vicon/"+frameName+"/"+frameName, t)
        t = tfl.getLatestCommonTime("/static_corner_4", "/vicon/"+frameName+"/"+frameName)
        position4, quaternion4 = tfl.lookupTransform("/static_corner_4", "/vicon/"+frameName+"/"+frameName, t)
        x = (position1[0]+position2[0]+position3[0]+position4[0])/4
        y = (position1[1]+position2[1]+position3[1]+position4[1])/4
        q = ((quaternion1[0]+quaternion2[0]+quaternion3[0]+quaternion4[0])/4,(quaternion1[1]+quaternion2[1]+quaternion3[1]+quaternion4[1])/4,(quaternion1[2]+quaternion2[2]+quaternion3[2]+quaternion4[2])/4,(quaternion1[3]+quaternion2[3]+quaternion3[3]+quaternion4[3])/4)
        euler = tf.transformations.euler_from_quaternion(q)    
        theta = euler[2]
        return x,y,theta
    except (tf.LookupException,tf.ConnectivityException, tf.ExtrapolationException):
        pass

def cmdZumo(index,v,w):
#Send commands to zumos
#Add paper reference
    l=0.09 #track width (meters)
    rv = v+0.5*l*w
    lv = v-0.5*l*w
    cur_speed = (rv+lv)/2
    if(abs(cur_speed)>0.5): #Scales velocity if asking a wheel to go faster than it actually can
        eef = 0.5/abs(cur_speed)
        rv = rv*eef
        lv = lv*eef
    velocities = str(index) + ':' + str(lv) + ',' + str(rv) + ';'
    socketIO.emit('getvelocity', velocities)

def refToZumo(index,x,y,theta,xRef,yRef,drx,dry,sp): 
#line following
#Add paper reference
    global timeBefore
    global thetaRefBefore
    dtime = time.time() - timeBefore[index-1]
    timeBefore[index-1] = time.time()
    b = 60 #7020
    eta = 0.25 #0.620
    vRef = sp
    thetaRef = math.atan2(dry,drx)
    wRef = (thetaRef-thetaRefBefore[index-1])/dtime
    thetaRefBefore[index-1] = thetaRef
    vRef = sp
    
    k1 = 2*eta*np.sqrt(np.power(wRef,2)+b*np.power(vRef,2))
    k2 = b
    k3 = k1

    thetaError =thetaRef - theta #the difference between the refTheta and actual theta
    if thetaError<-3.1415:
        thetaError = 2*3.1415+thetaError
    if thetaError>3.1415: # fix bad path instances by ignoring them
        thetaError = 0

    v = vRef*math.cos(thetaError) + k1 * (math.cos(theta)*(xRef-x)+math.sin(theta)*(yRef-y))
    # w = wRef + k2 * np.sign(vRef) * (math.cos(theta)*(yRef-y)-math.sin(theta)*(xRef-x)) + k3*(thetaError)
    if thetaError==0:
        w = wRef + (k2 * vRef ) * (-math.sin(theta)*(xRef-x)+math.cos(theta)*(yRef-y)) + k3*(thetaError)
    else: 
        w = wRef + (k2 * vRef * (math.sin(thetaError)/(thetaError))) * (-math.sin(theta)*(xRef-x)+math.cos(theta)*(yRef-y)) + k3*(thetaError)
    if abs(w)>4: # fix bad path instances, but this causes wide swings - cars can keep running though
        w = np.sign(w)*4

    #print "index: " + str(index) + "   v: " + str(v) + "   w: " + str(w) 

    cmdZumo(index,v,w)
    return v,w


	
def callback(msg): # string contains: i,xRef,yRef
    global data
    refString = msg.data #.data turns the rosmsg into a String (not related to the  data collection matrix "global data")
    refList = refString.split(",")
    index = int(refList[0])
    xRef = float(refList[1])
    yRef = float(refList[2])
    drx = float(refList[3])
    dry = float(refList[4])
    sp = float(refList[5])
    x,y,theta = getViconPos(index)
    v,w = refToZumo(index,x,y,theta,xRef,yRef,drx,dry,sp)
    data = dataCollection(data,index,x,y,v)
    data = dataCollection(data,index+10,xRef,yRef,sp)

def main():
    global tfl
    rospy.init_node('zumo_go') #getRefs')
    tfl = tf.TransformListener()
    rospy.Subscriber("/ZumoRefs",String,callback)
    rospy.spin()

if __name__ == '__main__':
    global data
    global timeBefore
    global thetaBefore
    global timeStart
    socketIO = SocketIO('192.168.1.245', 5001, LoggingNamespace)
    socketIO.on('connect',on_connect)
    data = ""
    timeBefore = [time.time()]*35
    thetaRefBefore = [0]*35
    timeStart = time.time()
    ''''try:
        main()
    except KeyboardInterrupt:
        print "Game Over :("
    finally:
        for i in range(20): #assumes twenty zumos maximum
            cmdZumo(i,0,0)
        saveFile(data)'''
    main()
    for i in range(36):
        cmdZumo(i,0,0)
    saveFile(data)
