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



# 2.31,-2.61
posx[32] = 2.6#0.35
posy[32] = 0.12#-4.3




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

    ax1.scatter(posx[32],posy[32],c='r',s=300)

    ax1.plot(xs0,ys0,'g')
    ax1.plot(xs1,ys1,'b')
    ax1.plot(xc0,yc0,'y')
    ax1.plot(xc1,yc1,'y')
    ax1.plot(xm,ym,'k')
    ax1.set_xlim([3.048,-3.048])
    ax1.set_ylim([1.524,-1.524])

def zumoThread(index, frameName, controlRate, path):
    global posx
    global posy
    x = posx[index]
    y = posy[index]
    status = path.GetSegmentIDNoCheck()#return the Path segment number 
    road_speed = 0.25 #speed limit on the road unless intervened by from the controller
    speed = road_speed #*1.145 #linear scaling factor 1.145 to achieve 0.30 actual

    nf = rospy.ServiceProxy('lf_grad', LineFollowing) #nf is the result of the LineFollowing service call which we name 'lf_grad', look in folder srv to see inputs and outputs
    nf_m = rospy.ServiceProxy('lf_merge', MergeControl)
    cur_t = time.time()

    dx = 0.0 #initialize gradient direction vectors
    dy = 0.0

    isControlled = False #initially not in the control region
    abcd = np.matrix([[0],[0],[0],[0]]) #initialize an RT_control matrix
    tinit = 0
    controlInfoPre = (None,None)

    while isRun:
        temp_t = time.time()
        x = x + dx*speed*(temp_t-cur_t) #Note temp_t and cur_t names seem to be backward
        y = y + dy*speed*(temp_t-cur_t)
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
            #Added to slow cars down around turns
            if status==2 or status==3 or status==5 or status==14 or status==15 or status==6 or status==10 or status==5 or status==14 or status==11 or status==13 or status==8:#making car go slower at arcs
                road_speed=0.27
            else:
                road_speed=0.27
            #####

            if res[0] == 0:
	        dx = res[1]
	        dy = res[2]
            elif res[0]==2:
                pass
            else:
	        print "Zumo "+str(index)+" Cannot Run NF."
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        controlInfo = path.CheckControl(x, y) #controlInfo can be 0,1,2 zero is in control region (L), one is in the merge region, and two is exiting the merge region. Check control checks if the position is > or < a specified transition condition
        if (controlInfo is not None) and (controlInfo != controlInfoPre): #if the position falls in the control region
            if controlInfo[0] == 0:
                rospy.wait_for_service('lf_merge') #if
                mf = nf_m(index,controlInfo[1],controlInfo[2],0,road_speed)
                if mf.isFirst: 
                    isControlled = False
                else:
                    isControlled = True
                    abcd = RT_control(time.time()-mf.tinit,mf.tmi-mf.tinit,0,mf.L,road_speed,road_speed)
                    tinit = mf.tinit
                    #print "Robot "+str(index)+": to->"+str(time.time()-mf.tinit)+" tmi->"+str(mf.tmi-mf.tinit)+" xi->0 xf->"+str(mf.L)+" v->"+str(cur_speed)+" vf->"+str(cur_speed)
                    #print "ABCD: "+str(abcd)
            elif controlInfo[0] == 2:
                isControlled = False
                rospy.wait_for_service('lf_merge')
                mf = nf_m(index,controlInfo[1],controlInfo[2],1,road_speed)
            elif controlInfo[0] == 1:
                isControlled = False
        controlInfoPre = controlInfo

        if not isControlled: 
            speed = road_speed #*1.145 #this is a correction that is added to the real robot control and is removed here because
        else:
            temps = 0.5*abcd[0]*(time.time()-tinit)*(time.time()-tinit)+abcd[1]*(time.time()-tinit)+abcd[2]
            ttemps = temps.item(0)
            speed = ttemps #*1.145
            #if speed >0.7:
                #speed = 0.7
        time.sleep(controlRate)

rospy.init_node('zumo_go', anonymous=True)#zumo_go is a node

pub = rospy.Publisher('/ZumoRefs', String, queue_size=1000)#ZumoRefs is a topic name 


#t16.start()

t18 = Thread(target = zumoThread, args = (32, "zumoTest32", 0.05, copy.deepcopy(Path.GetDefaultPath(23))))
t18.start()

ani = animation.FuncAnimation(fig, animate, interval=100)
plt.axis('equal')
plt.show()

isRun = False
