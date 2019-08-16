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


#road 2 lower

posx[17] = -1.43
posy[17] = 0.25
posx[18] = -1.7
posy[18] = -.144
posx[19] = -2.183
posy[19] = -.23



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
    global xs0#long road
    global ys0#Long road
    global xs1
    global ys1

    ax1.clear()

	#the dot at 0,0 is caused by having extra indexes here that don't corresspond to an actual car in the sim.

    ax1.scatter(posx[17],posy[17],c='r',s=300)
    ax1.scatter(posx[18],posy[18],c='r',s=300)
    ax1.scatter(posx[19],posy[19],c='r',s=300)


    ax1.plot(xs0,ys0,'g')
    ax1.plot(xs1,ys1,'b')
    ax1.plot(xc0,yc0,'y')
    ax1.plot(xc1,yc1,'y')
    ax1.plot(xm,ym,'k')
    ax1.set_xlim([3.048,-3.048])
    ax1.set_ylim([1.524,-4.5])

def zumoThread(index, frameName, controlRate, path):
    global posx
    global posy
    x = posx[index]
    y = posy[index]
    status = path.GetSegmentIDNoCheck()#return the Path segment number 
    road_speed = 0.25 #speed limit on the road unless intervened by from the controller
    speed = road_speed #*1.145 #linear scaling factor 1.145 to achieve 0.30 actual

    nf = rospy.ServiceProxy('lf_grad', LineFollowing) #nf is the result of the LineFollowing service call which we name 'lf_grad', look in folder srv to see inputs and outputs
    cur_t = time.time()

    dx = 0.0 #initialize gradient direction vectors
    dy = 0.0

    isControlled = False #initially not in the control region
    abcd = np.matrix([[0],[0],[0],[0]]) #initialize an RT_control matrix
    tinit = 0
    controlInfoPre = (None,None)

    while isRun:
        temp_t = time.time()
        del_t=temp_t-cur_t
        x = x + dx*speed*(del_t) #Note temp_t and cur_t names seem to be backward
        y = y + dy*speed*(del_t)
        posx[index] = x #index refers to the car number, update that cars x position
        posy[index] = y
        talker(index,x,y,dx,dy,speed) #publish the cars number, its position, the desired vector to the next position and its desired speed
        
        cur_t = temp_t
        
        rospy.wait_for_service('lf_grad')
        try:
            resp = nf(status, x, y) # from the LineFollowing service, the output res,dx,dy is saved as (ros object?) resp
            res = [resp.res, resp.dx, resp.dy] # turn the ros object resp into a "useable" vector
            status = path.GetSegmentID(x,y) #
            ######
            #Calculate Car Speed Desired
            #Need Velocities
            speed=idm(headway,velocities[index],velocities[leader],vmax)d*del_t+speed
            #####

            if res[0] == 0:
	        dx = res[1]
	        dy = res[2]
            elif res[0]==2:
                pass
            else:cd catkin_ws 
catkin_make
	        printcd catkin_ws 
catkin_make
        except rocd catkin_ws 
catkin_make
            printcd catkin_ws 
catkin_make
        if posx[icd catkin_ws 
catkin_makes = #whatever A102 ends up being
           yieldicd catkin_ws 
catkin_make
        else
           yieldicd catkin_ws 
catkin_make
        if yieldicd catkin_ws 
catkin_make
           speed=trafficYield(x_min,x_max,y_min,y_max,road_speed,posx,posy)
        time.sleep(controlRate)

rospy.init_node('zumo_go', anonymous=True)#zumo_go is a node

pub = rospy.Publisher('/ZumoRefs', String, queue_size=1000)#ZumoRefs is a topic name 

t11 = Thread(target = zumoThread, args = (17, "zumoTest17", 0.05, copy.deepcopy(Path.GetDefaultPath(27))))
t11.start()
t12 = Thread(target = zumoThread, args = (18, "zumoTest18", 0.05, copy.deepcopy(Path.GetDefaultPath(27))))
t12.start()
t13 = Thread(target = zumoThread, args = (19, "zumoTest19", 0.05, copy.deepcopy(Path.GetDefaultPath(27))))
t13.start()

ani = animation.FuncAnimation(fig, animate, interval=100)
plt.axis('equal')
plt.show()

isRun = False
