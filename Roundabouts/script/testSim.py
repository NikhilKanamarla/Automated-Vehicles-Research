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
#import Path
import copy

isRun = True

style.use('fivethirtyeight')

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)

#t = 0
xs0,ys0,xs1,ys1,xc0,yc0,xc1,yc1,xm,ym = BuildPath(0.05)

posx = [0]*36
posy = [0]*36

#For path index 13 Starting at S68

posx[12] = -0.4
posy[12] = -1.2972

posx[28] =-0.9
posy[28] =-1.2972

#posx[10] =-0.9
#posy[10] =-1.2972

posx[14] = -1.4
posy[14] = -1.2972

#posx[12] = -1.9
#posy[12] = -1.2972


#posx[10] =-2.5
#posy[10] =-1.2972





##############################################################333
#For path index 12


#posx[32] = 2.3114 #test
#posy[32] = -3.2

posx[32] = 2.3114
posy[32] = -2.65

posx[28] = 2.3114
posy[28] = -2.05

#posx[21] = 2.3114
#posy[21] = -1.45

posx[34] = 2.3114
posy[34] = -0.8



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

    ax1.scatter(posx[1],posy[1],c='b',s=210)
    ax1.scatter(posx[2],posy[2],c='g',s=210)
    ax1.scatter(posx[4],posy[4],c='g',s=210)
    ax1.scatter(posx[12],posy[12],c='g',s=210)
    ax1.scatter(posx[10],posy[10],c='g',s=210)

    ax1.scatter(posx[11],posy[11],c='r',s=210)
    ax1.scatter(posx[19],posy[19],c='r',s=210)
    #ax1.scatter(posx[22],posy[22],c='r',s=210)
    ax1.scatter(posx[23],posy[23],c='r',s=210)
    ax1.scatter(posx[21],posy[21],c='r',s=210)

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
            ''
            if status==32 or status==30 or status==28 or status==23 or status==38 or status==36 or status==37:#making car go slower at turning
                road_speed=0.23
            else:
                road_speed=0.25
            ''
########################################Test
 #           if index !=15:
  #              road_speed=0
#############################################

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


##Path 13
t1 = Thread(target = zumoThread, args = (9, "zumoTest9", rate, copy.deepcopy(GetDefaultPath(13))))
t1.start()

t2 = Thread(target = zumoThread, args = (10, "zumoTest10", rate, copy.deepcopy(GetDefaultPath(13))))
t2.start()


t3 = Thread(target = zumoThread, args = (14, "zumoTest14", rate, copy.deepcopy(GetDefaultPath(13))))
t3.start()


#t4 = Thread(target = zumoThread, args = (5, "zumoTest5", rate, copy.deepcopy(GetDefaultPath(13))))
#t4.start()

t5 = Thread(target = zumoThread, args = (12, "zumoTest12", rate, copy.deepcopy(GetDefaultPath(13))))
t5.start()


##Path 12


t11 = Thread(target = zumoThread, args = (28, "zumoTest28", rate, copy.deepcopy(GetDefaultPath(12))))
t11.start()

t12 = Thread(target = zumoThread, args = (32, "zumoTest32", rate, copy.deepcopy(GetDefaultPath(12))))
t12.start()

t13 = Thread(target = zumoThread, args = (34, "zumoTest34", rate, copy.deepcopy(GetDefaultPath(12))))
t13.start()

t17 = Thread(target = zumoThread, args = (21, "zumoTest21", rate, copy.deepcopy(GetDefaultPath(12))))
t17.start()

#t18 = Thread(target = zumoThread, args = (22, "zumoTest22", rate, copy.deepcopy(GetDefaultPath(12))))
#t18.start()



ani = animation.FuncAnimation(fig, animate, interval=100)
plt.axis('equal')
plt.show()

isRun = False
