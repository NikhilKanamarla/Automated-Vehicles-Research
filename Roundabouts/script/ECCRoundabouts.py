#!/usr/bin/env python

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
from matplotlib import style
from Points import BuildPath
from std_msgs.msg import String
from Path import GetDefaultPath
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
#Thread Variables
rate = .05
import Path
import copy

isRun = True

style.use('fivethirtyeight')

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)

#t = 0
xs0,ys0,xs1,ys1,xc0,yc0,xc1,yc1,xm,ym = BuildPath(0.05)

posx = [0]*36
posy = [0]*36


##############################################################
#For path index 8

posx[1] = 2.2
posy[1] = 0.01

posx[2] = 2.85
posy[2] = 0.6

posx[3] = 2.4
posy[3] = 1.2

posx[4] = 1.7
posy[4] = 0.25

#path 23
posx[5] = 1.2
posy[5] = 0.8

posx[6] = 0.75
posy[6] = 0.8

posx[7] = 0.25
posy[7] = 0.15

posx[8] = 1
posy[8] = -1

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

    ax1.scatter(posx[1],posy[1],c='b',s=300)
    ax1.scatter(posx[2],posy[2],c='b',s=300)
    ax1.scatter(posx[3],posy[3],c='b',s=300)
    ax1.scatter(posx[4],posy[4],c='b',s=300)



    ax1.scatter(posx[5],posy[5],c='r',s=300)
    ax1.scatter(posx[6],posy[6],c='r',s=300)
    ax1.scatter(posx[7],posy[7],c='r',s=300)
    ax1.scatter(posx[8],posy[8],c='r',s=300)

    ax1.plot(xs0,ys0,'g')
    ax1.plot(xs1,ys1,'b')
    ax1.plot(xc0,yc0,'y')
    ax1.plot(xc1,yc1,'y')
    ax1.plot(xm,ym,'k')
    ax1.set_xlim([3.048,-3.548])
    ax1.set_ylim([1.524,-5]) #([1.524,-1.524])

def zumoThread(index, frameName, controlRate, path):
    global posx
    global posy
    x = posx[index]
    y = posy[index]
    status = path.GetSegmentIDNoCheck()
    road_speed = 0.25
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

		
            road_speed=0.27

            if res[0] == 0:
	          dx = res[1]
	          dy = res[2]
            elif res[0]==2:
                pass
            else:
	        print "Zumo "+str(index)+" Cannot Run NF."
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        controlInfo = path.CheckControl(x, y)
        if (controlInfo is not None) and (controlInfo != controlInfoPre):
            if controlInfo[0] == 0:
                rospy.wait_for_service('lf_merge')
                mf = nf_m(index,controlInfo[1],controlInfo[2],0,road_speed)
                if mf.isFirst: 
                    isControlled = False
                else:
                    isControlled = True
                    abcd = RT_control(time.time()-mf.tinit,mf.tmi-mf.tinit,0,mf.L,road_speed,road_speed)
                    tinit = mf.tinit
            elif controlInfo[0] == 2:
                isControlled = False
                rospy.wait_for_service('lf_merge')
                mf = nf_m(index,controlInfo[1],controlInfo[2],1,road_speed)
            elif controlInfo[0] == 1:
                isControlled = False
        controlInfoPre = controlInfo

		 	 if isControlled: 

           		 temps = 0.5*abcd[0]*(time.time()-tinit)**2+abcd[1]*(time.time()-tinit)+abcd[2]
	    #print temps
            	ttemps = temps.item(0)
            	speed =ttemps #*1.145

        	else:

            	speed = road_speed 

        		time.sleep(controlRate)



rospy.init_node('zumo_go', anonymous=True)

pub = rospy.Publisher('/ZumoRefs', String, queue_size=1000)


##Path 17

t1 = Thread(target = zumoThread, args = (1, "zumoTest1", rate, copy.deepcopy(GetDefaultPath(8))))
t1.start()

t2 = Thread(target = zumoThread, args = (2, "zumoTest2", rate, copy.deepcopy(GetDefaultPath(8))))
t2.start()

t3 = Thread(target = zumoThread, args = (3, "zumoTest3", rate, copy.deepcopy(GetDefaultPath(8))))
t3.start()

t4 = Thread(target = zumoThread, args = (4, "zumoTest4", rate, copy.deepcopy(GetDefaultPath(8))))
t4.start()

t5 = Thread(target = zumoThread, args = (5, "zumoTest5", rate, copy.deepcopy(GetDefaultPath(23))))
t5.start()

t6 = Thread(target = zumoThread, args = (6, "zumoTest6", rate, copy.deepcopy(GetDefaultPath(23))))
t6.start()

t7 = Thread(target = zumoThread, args = (7, "zumoTest7", rate, copy.deepcopy(GetDefaultPath(23))))
t7.start()

t8 = Thread(target = zumoThread, args = (8, "zumoTest8", rate, copy.deepcopy(GetDefaultPath(23))))
t8.start()



ani = animation.FuncAnimation(fig, animate, interval=100)
plt.axis('equal')
plt.show()

isRun = False
