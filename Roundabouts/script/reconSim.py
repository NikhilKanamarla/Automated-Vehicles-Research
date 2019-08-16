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
from getViconPos import getViconPos

isRun = True

style.use('fivethirtyeight')

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)

#t = 0
xs0,ys0,xs1,ys1,xc0,yc0,xc1,yc1,xm,ym = BuildPath(0.05)
#Set Indicies
index1=17
name1="zumoTest17"
index2=10
name2="zumoTest10"
index3=14
name3="zumoTest14"
index4=15
name4="zumoTest15"
index5=22
name5="zumoTest1522"
#index6=35
#name6="zumoTest35"

posx = [0]*36
posy = [0]*36

#For path index 13 Starting at S68
#initial position for the leader 
posx[index1] = 1.2
posy[index1] = -1.15

posx[index2] = 1.5
posy[index2] = -1.15

posx[index3] = 1.8
posy[index3] = -1.15

posx[index4] = 2.1
posy[index4] = -1.15

posx[index5] = 2.4
posy[index5] = -1.15

#posx[index6] = 2.4
#posy[index6] = -1.15

def isInside(name,maxX,maxY,minX,minY): #Function returns if Vicon sees an object inside a fence
    try:
        [x,y,theta] = getViconPos(name)
        #print "x value is " + str(x)
        #print "y value is " + str(y)
        if x<=maxX and x >=minX and y<=maxY and y>=minY:
           return True
        else:
           return False
    except:
       return False


def getViconPos(frameName):
    global tfl
    try:
        #print (frameName, " is of type ",type(frameName))
        t = tfl.getLatestCommonTime("/static_corner_1", "/vicon/"+frameName+"/"+frameName)
        position1, quaternion1 = tfl.lookupTransform("/static_corner_1", "/vicon/"+frameName+"/"+frameName, t)
        #print frameName
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

    ax1.scatter(posx[index1],posy[index1],c='b',s=210)
    ax1.scatter(posx[index2],posy[index2],c='r',s=210)
    ax1.scatter(posx[index3],posy[index3],c='g',s=210)
    ax1.scatter(posx[index4],posy[index4],c='m',s=210)
    ax1.scatter(posx[index5],posy[index5],c='k',s=210)


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
    indicies=[index1,index2,index3,index4,index5]
    i_index=indicies.index(index)
    status = path.GetSegmentIDNoCheck()
    road_speed = 0.15
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
        #Vicon Calls For recon drone and lead truck
        #[xt,yt,tt]=getViconPos(9)
        truckIn = isInside(name1,.6,-1.0,.3,-1.3)
        UAVin = isInside("cf1",.6,-1.0,.3,-1.3)
        print("truckIn",truckIn,"UAVin",UAVin)
        if truckIn and not UAVin:
           road_speed = 0;
        else:
           road_speed = .15;
 
        x = x + dx*speed*(temp_t-cur_t) #Note temp_t and cur_t names seem to be backward
        y = y + dy*speed*(temp_t-cur_t)
        posx[index] = x #index refers to the car number, update that cars x position
        posy[index] = y

        talker(index,x,y,dx,dy,speed)
        
        cur_t = temp_t
        
        rospy.wait_for_service('lf_grad')
        try:
            resp = nf(status, x, y)
            res = [resp.res, resp.dx, resp.dy]
            status = path.GetSegmentID(x,y)
            ''
            if status==45:#making car go stop
                road_speed=0
            #else:
                #road_speed=0.25
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



global pub
pub = rospy.Publisher('/ZumoRefs', String, queue_size=1000)
def main():
    global tfl
    tfl = tf.TransformListener()
    rospy.init_node('dummy')
    ##Path 13
    t1 = Thread(target = zumoThread, args = (index1, name1, rate, copy.deepcopy(GetDefaultPath(26))))
    t1.start()
    t2 = Thread(target = zumoThread, args = (index2, name2, rate, copy.deepcopy(GetDefaultPath(26))))
    t2.start()
    t3 = Thread(target = zumoThread, args = (index3, name3, rate, copy.deepcopy(GetDefaultPath(26))))
    t3.start()

    t4 = Thread(target = zumoThread, args = (index4, name4, rate, copy.deepcopy(GetDefaultPath(26))))
    t4.start()

    t5 = Thread(target = zumoThread, args = (index5, name5, rate, copy.deepcopy(GetDefaultPath(26))))
    t5.start()

#t6 = Thread(target = zumoThread, args = (index6, name6, rate, copy.deepcopy(GetDefaultPath(26))))
#t6.start()

if __name__ == '__main__':

    main()
ani = animation.FuncAnimation(fig, animate, interval=100)
plt.axis('equal')
plt.show()

isRun = False
