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

posx = [0]*35 #changed this in Chris Version to make longer array
posy = [0]*35
#road 1 upper

'''
posx[11] = 0.2
posy[11] = -1.3
posx[12] = 0.2
posy[12] = -1.75
posx[13] = 0.2
posy[13] = -2.2
posx[14] = 0.2
posy[14] = -2.65
posx[15] = 0.2
posy[15] = -3.1

'''
#road1 Car positions (ON SECONDARY ROAD)


posx[5] = 0.4
posy[5] = -1
posx[10] = 0.7
posy[10] = -1
posx[33] = 1
posy[33] = -1

posx[31] = 1.3
posy[31] = -1
'''
posx[32] = 1.6
posy[32] = -1
'''

'''

#road 2 lower

posx[6] = -1.43
posy[6] = 0.25
posx[7] = -1.7
posy[7] = -.144
posx[8] = -2.183
posy[8] = -.23
posx[9] = -2.577
posy[9] = .0853
posx[10] = -2.599
posy[10] =.578
'''
#road 2 car positions NOT secondary road
posx[14] = -1.43
posy[14] = 0.25

posx[26] = -1.55
posy[26] = -.03

#posx[21] = -1.8
#posy[21] = -0.22


#posx[15] = -1.85
#posy[15] = -0.22
'''
posx[22] = -2.34
posy[22] = -0.166
'''


global inMerge
inMerge = [0]*35 #also changed in chris' version, was orig 20
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

    ax1.scatter(posx[5],posy[5],c='r',s=200)
    ax1.scatter(posx[10],posy[10],c='r',s=200)
    ax1.scatter(posx[33],posy[33],c='r',s=200)
    ax1.scatter(posx[31],posy[31],c='r',s=200)
    #ax1.scatter(posx[32],posy[32],c='r',s=300)

    ax1.scatter(posx[14],posy[14],c='r',s=200)
    ax1.scatter(posx[26],posy[26],c='r',s=200)
    #ax1.scatter(posx[21],posy[21],c='r',s=200)


    #ax1.scatter(posx[22],posy[22],c='r',s=300)

    #ax1.scatter(posx[15],posy[15],c='r',s=300)

	#c is colors, s is size

    ax1.plot(xs0,ys0,'g')
    ax1.plot(xs1,ys1,'b')
    ax1.plot(xc0,yc0,'y')
    ax1.plot(xc1,yc1,'y')
    ax1.plot(xm,ym,'k')
    ax1.set_xlim([3.048,-3.048])
    ax1.set_ylim([1.524,-1.524])

def checkMerge(index,status,x): #not in mergeSim, could be the yielding part?
#all the cars on secondary road were hardcoded to have an index less than 10
#to fix, replace index <=10 with the cars on the secondary road
    global inMerge
    if (index == 14 or index == 12 or index == 15 or index == 22 or index == 26) and status == 4 and x>0 and x<0.72:
        inMerge[index] = 1
    elif (index == 14 or index == 12 or index == 15 or index == 22 or index == 26):
        inMerge[index] = 0
    else:
        inMerge[index] = 0
    return inMerge

def zumoThread(index, frameName, controlRate, path):
    global posx
    global posy
    global inMerge
    global numRobots
    x = posx[index]
    y = posy[index]
    status = path.GetSegmentIDNoCheck()
    road_speed = 0.3
    speed = road_speed #*1.145 #linear scaling factor 1.145 to achieve 0.30 actual

    nf = rospy.ServiceProxy('lf_grad', LineFollowing)
    nf_m = rospy.ServiceProxy('lf_merge', MergeControl)

    cur_t = time.time()

    dx = 0.0
    dy = 0.0

    isControlled = False
    abcd = np.matrix([[0],[0],[0],[0]])
    tinit = 0
    controlInfoPre = (None,None)

    inWaiting = 0

    while isRun:
        temp_t = time.time()
        x = x + dx*speed*(temp_t-cur_t)
        y = y + dy*speed*(temp_t-cur_t)
        posx[index] = x
        posy[index] = y
        talker(index,x,y,dx,dy,speed)
        
        cur_t = temp_t
        
        rospy.wait_for_service('lf_grad')
        try:
            resp = nf(status, x, y)
            res = [resp.res, resp.dx, resp.dy]
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


	#this part of zumoThread is unique to baselineSim, not in mergeSim
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

        '''waitDist[index] = 0.6 + 0.4*(np.amax(inWaiting[index]))
        print waitDist

        if status == 10 and y>=waitDist: # and np.sum(inMerge)>0:
            speed = 0
            if inWaiting[index] == 0:
                inWaiting[index] = np.amax(inWaiting)+1
            print inWaiting
        else:
            if inWaiting[index]==1:
                for j in range(len(inWaiting)):
                    if inWaiting[j]>0:
                        inWaiting[j] = inWaiting[j]-1
            else:
                inWaiting[index] = 0
            speed = road_speed #*1.145 #this is a correction that is added to the real robot control''' 
        
        time.sleep(controlRate)

rospy.init_node('zumo_go', anonymous=True)

pub = rospy.Publisher('/ZumoRefs', String, queue_size=1000)
'''
#zipper starting positions
t11 = Thread(target = zumoThread, args = (11, "zumoTest11", 0.05, copy.deepcopy(Path.GetDefaultPath(1))))
t11.start()
t12 = Thread(target = zumoThread, args = (12, "zumoTest13", 0.05, copy.deepcopy(Path.GetDefaultPath(1))))
t12.start()
t13 = Thread(target = zumoThread, args = (13, "zumoTest13", 0.05, copy.deepcopy(Path.GetDefaultPath(1))))
t13.start()
t14 = Thread(target = zumoThread, args = (14, "zumoTest14", 0.05, copy.deepcopy(Path.GetDefaultPath(1))))
t14.start()
t15 = Thread(target = zumoThread, args = (15, "zumoTest15", 0.05, copy.deepcopy(Path.GetDefaultPath(1))))
t15.start()
t6 = Thread(target = zumoThread, args = (6, "zumoTest6", 0.05, copy.deepcopy(Path.GetDefaultPath(0))))
t6.start()
t7 = Thread(target = zumoThread, args = (7, "zumoTest7", 0.05, copy.deepcopy(Path.GetDefaultPath(0))))
t7.start()
t8 = Thread(target = zumoThread, args = (8, "zumoTest8", 0.05, copy.deepcopy(Path.GetDefaultPath(0))))
t8.start()
t9 = Thread(target = zumoThread, args = (9, "zumoTest9", 0.05, copy.deepcopy(Path.GetDefaultPath(0))))
t9.start()
t10 = Thread(target = zumoThread, args = (10, "zumoTest10", 0.05, copy.deepcopy(Path.GetDefaultPath(0))))
t10.start()
'''
#batt starting positions

t11 = Thread(target = zumoThread, args = (5, "zumoTest5", 0.05, copy.deepcopy(Path.GetDefaultPath(8))))
t11.start()
t12 = Thread(target = zumoThread, args = (10, "zumoTest10", 0.05, copy.deepcopy(Path.GetDefaultPath(8))))
t12.start()
t13 = Thread(target = zumoThread, args = (31, "zumoTest31", 0.05, copy.deepcopy(Path.GetDefaultPath(8))))
t13.start()
t14 = Thread(target = zumoThread, args = (32, "zumoTest32", 0.05, copy.deepcopy(Path.GetDefaultPath(8))))
t14.start()
t15 = Thread(target = zumoThread, args = (33, "zumoTest33", 0.05, copy.deepcopy(Path.GetDefaultPath(8))))
t15.start()
t6 = Thread(target = zumoThread, args = (14, "zumoTest12", 0.05, copy.deepcopy(Path.GetDefaultPath(0))))
t6.start()
t7 = Thread(target = zumoThread, args = (21, "zumoTest21", 0.05, copy.deepcopy(Path.GetDefaultPath(0))))
t7.start()
t8 = Thread(target = zumoThread, args = (15, "zumoTest15", 0.05, copy.deepcopy(Path.GetDefaultPath(0))))
t8.start()
t9 = Thread(target = zumoThread, args = (22, "zumoTes22", 0.05, copy.deepcopy(Path.GetDefaultPath(0))))
t9.start()
t10 = Thread(target = zumoThread, args = (26, "zumoTest26", 0.05, copy.deepcopy(Path.GetDefaultPath(0))))
t10.start()


ani = animation.FuncAnimation(fig, animate, interval=100)
plt.axis('equal')
plt.show()

isRun = False
