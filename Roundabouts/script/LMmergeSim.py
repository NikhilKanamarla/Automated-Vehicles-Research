#!/usr/bin/env python

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
from matplotlib import style
from Points import BuildPath
from std_msgs.msg import String

import rospy
import tf
import time
import datetime ##
import numpy as np
from numpy.linalg import inv,pinv
from threading import Thread, Lock
from geometry_msgs.msg import TransformStamped
from line_following.srv import *
from socketIO_client import SocketIO, LoggingNamespace


import Path
import copy
isRun = True

style.use('fivethirtyeight')

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)

#t = 0
xs0,ys0,xs1,ys1,xc0,yc0,xc1,yc1,xm,ym = BuildPath(0.05)

posx = [0]*35
posy = [0]*35
#road 1 upper

#road1 Car positions (secondary road)

posx[11] = 0.444
posy[11] = -1

#road 2 car positions (main road)
posx[6] = -1.43
posy[6] = 0.25


global inMerge
inMerge = [0]*20
global numRobots
numRobots = 0

def talker(index,x,y,drx,dry,sp):
    global pub
    pub.publish(str(index)+","+str(x)+","+str(y)+","+str(drx)+","+str(dry)+","+str(sp))

def RT_control(to,tmi,xi,xf,v,vf):
    A_mtx = np.matrix([[to**3/6,to**2/2,to,1],[to**2/2,to,1,0],[tmi**3/6,tmi**2/2,tmi,1],[tmi**2/2,tmi,1,0]])
    Y_mtx = np.matrix([[xi],[v],[xf],[vf]])

    A_aux = np.transpose(A_mtx)*A_mtx
    X_mtx = pinv(A_aux)*np.transpose(A_mtx)*Y_mtx

    return X_mtx

def animate(i):
    #global t
    global xs0
    global ys0
    global xs1
    global ys1

    ax1.clear()
    ax1.scatter(posx[11],posy[11],c='b',s=300)
    ax1.scatter(posx[6],posy[6],c='r',s=300)
    ax1.plot(xs0,ys0,'g')
    ax1.plot(xs1,ys1,'b')
    ax1.plot(xc0,yc0,'y')
    ax1.plot(xc1,yc1,'y')
    ax1.plot(xm,ym,'k')
    ax1.set_xlim([3.048,-3.048])
    ax1.set_ylim([1.524,-1.524])


def checkMerge(index,status,x):
    global inMerge
    if index <= 10 and status == 4 and x>0 and x<0.72:
        inMerge[index] = 1
    elif index <= 10:
        inMerge[index] = 0
    else:
        inMerge[index] = 0
    return inMerge

def zumoThread(index, frameName, controlRate, path):
    global posx
    global posy
    global inMerge
    global numRobots
   
    u = 0.27   # initial velocity
    a1 = -0.1  # deceleration
    a2 = 0.1  # acceleration

    x = posx[index]
    y = posy[index]
    status = path.GetSegmentIDNoCheck()  

   # print ("the value of status and isrun is ", status, isRun)

    nf = rospy.ServiceProxy('lf_grad', LineFollowing)
    nf_m = rospy.ServiceProxy('lf_merge', MergeControl)
    cur_t = time.time() # system time is taken as current time
    dx = 0.0
    dy = 0.0
    
    speed = 0.3
    inWaiting = 0

    while isRun:
        print ("the value of status in loop is ", status)
        temp_t = time.time() # time at every instance. as long as the while loop runs
	road_speed = u + a1*temp_t # constant decelerating speed
        speed = road_speed #*1.145 #linear scaling factor 1.145 to achieve 0.30 actual
	
        #x = x + speed*(temp_t-cur_t)
        #y = y + speed*(temp_t-cur_t)
        
        x = x + dx*speed*(temp_t-cur_t) 
        y = y + dy*speed*(temp_t-cur_t)
	print('x position',x)
        posx[index] = x
        posy[index] = y
        talker(index,x,y,dx,dy,speed)
        
        cur_t = temp_t
        
        rospy.wait_for_service('lf_grad')
        try:
            resp = nf(status, x, y)
            res = [resp.res, resp.dx, resp.dy]
            print ("value of res is",res)
            status = path.GetSegmentID(x,y)
            #print str(index) + " is on segment " + str(status)

            inMerge = checkMerge(index,status,x) #################
            #print np.sum(inMerge)

            if res[0] == 0:
	        dx = res[1]
	        dy = res[2]
            elif res[0]==2:
                pass
            else:
	        print "Zumo "+str(index)+" Cannot Run NF."
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        if (status == 10 or status == 9) and y>0.52-0.32*numRobots and np.sum(inMerge)>0 and inWaiting == 0:
            speed = 0
            numRobots = numRobots+1
            inWaiting = 1
            print numRobots
        elif (status == 10 or status == 9) and y>0.52-0.32*numRobots and np.sum(inMerge)>0:
            speed = 0
            print numRobots
        elif not np.sum(inMerge)>0:
            speed = road_speed
            numRobots = 0
            inWaiting = 0
            print numRobots

        time.sleep(controlRate)

rospy.init_node('zumo_go', anonymous=True)

pub = rospy.Publisher('/ZumoRefs', String, queue_size=1000)

#batt starting positions

t11 = Thread(target = zumoThread, args = (11, "zumoTest11", 0.05, copy.deepcopy(Path.GetDefaultPath(8))))
t11.start()
t6 = Thread(target = zumoThread, args = (6, "zumoTest6", 0.05, copy.deepcopy(Path.GetDefaultPath(0))))
t6.start()


ani = animation.FuncAnimation(fig, animate, interval=100)
plt.axis('equal')
plt.show()

isRun = False
