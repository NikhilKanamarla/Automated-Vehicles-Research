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


##############################################################
#For path index 17


#posx[1] = 2.3114
#posy[1] = -3.2

#posx[2] = 2.3114
#posy[2] = -2.65

#posx[30] = 2.3114
#posy[30] = -2.05

#posx[2] = 2.3114
#posy[2] = -1.45

#posx[4] = 2.3114
#posy[4] = -0.8

#For path index15

#posx[7] = 0.8
#posy[7] = -1.2972

#posx[9] =0.3
#posy[9] = -1.2972

#posx[10] = -0.2
#posy[10] = -1.2972


#for path index 16

#posx[16] = -1.97
#posy[16] = -2.2

#posx[13] = -1.97
#posy[13] = -2.7

#posx[14] = -1.97
#posy[14] = -3.2

#posx[15] = -1.97
#posy[15] = -3.5

#for path index 18

#posx[21] = 2
#posy[21] = -3.302

#posx[20] = 1.5
#posy[20] = -3.302

#posx[19] = 1
#posy[19] = -3.302

#for path index 19

#posx[18] = -0.3
#posy[18] = -3.302

#posx[17] = -0.8
#posy[17] = -3.302

#posx[16] = -1.3
#posy[16] = -3.302




#lower road S94
posx[30] = -.7
posy[30] =1.2
#posx[23] = -0.2
#posy[23] = 1.2
#posx[24] = 0.3
#posy[24] = 1.2
#posx[25] = .8
#posy[25] = 1.2
#posx[26] = 1.3
#posy[26] = 1.2

#lower road S95 s91 s89

#posx[27] = 1.3
#posy[27] =0.7616
#posx[16] = 0.8
#posy[16] = 0.7616
#posx[29] = 0.3
#posy[29] = 0.7616
#posx[30] = -0.2
#posy[30] =0.7616
#posx[31] = -0.7
#posy[31] = 0.7616

# lower road  right ARC 105 101 100 
#posx[32] = -1.43
#posy[32] = 0.25
#posx[33] = -1.7
#posy[33] = -.144
#posx[34] = -2.183
#posy[34] = -.23

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

    #ax1.scatter(posx[30],posy[30],c='b',s=410)
    #ax1.scatter(posx[2],posy[2],c='g',s=210)
    #ax1.scatter(posx[3],posy[3],c='g',s=210)
    #ax1.scatter(posx[4],posy[4],c='g',s=210)
    #ax1.scatter(posx[5],posy[5],c='g',s=210)
    #ax1.scatter(posx[7],posy[7],c='g',s=210)
    #ax1.scatter(posx[9],posy[9],c='g',s=210)
    #ax1.scatter(posx[10],posy[10],c='g',s=210)
    #ax1.scatter(posx[12],posy[12],c='g',s=210)
    #ax1.scatter(posx[13],posy[13],c='g',s=210)
    #ax1.scatter(posx[14],posy[14],c='g',s=210)
    #ax1.scatter(posx[15],posy[15],c='g',s=210)

    #ax1.scatter(posx[17],posy[17],c='g',s=210)
    #ax1.scatter(posx[18],posy[18],c='g',s=210)
    #ax1.scatter(posx[19],posy[19],c='g',s=210)
    #ax1.scatter(posx[20],posy[20],c='g',s=210)
    #ax1.scatter(posx[21],posy[21],c='g',s=210)
    ax1.scatter(posx[30],posy[30],c='r',s=300)
    #ax1.scatter(posx[23],posy[23],c='r',s=300)
    #ax1.scatter(posx[24],posy[24],c='r',s=300)
    #ax1.scatter(posx[25],posy[25],c='r',s=300)
    #ax1.scatter(posx[26],posy[26],c='r',s=300)
    #ax1.scatter(posx[27],posy[27],c='r',s=300)
    #ax1.scatter(posx[16],posy[16],c='r',s=300)
    #ax1.scatter(posx[29],posy[29],c='r',s=300)
    #ax1.scatter(posx[30],posy[30],c='r',s=300)
    #ax1.scatter(posx[31],posy[31],c='r',s=300)
    #ax1.scatter(posx[32],posy[32],c='r',s=300)
    #ax1.scatter(posx[33],posy[33],c='r',s=300)
    #ax1.scatter(posx[34],posy[34],c='r',s=300)

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


##Path 17

#t30 = Thread(target = zumoThread, args = (30, "zumoTest30", rate, copy.deepcopy(GetDefaultPath(17))))
#t30.start()

#t14 = Thread(target = zumoThread, args = (2, "zumoTest2", rate, copy.deepcopy(GetDefaultPath(17))))
#t14.start()

#t15 = Thread(target = zumoThread, args = (3, "zumoTest3", rate, copy.deepcopy(GetDefaultPath(17))))
#t15.start()

#t16 = Thread(target = zumoThread, args = (4, "zumoTest4", rate, copy.deepcopy(GetDefaultPath(17))))
#t16.start()

#t17 = Thread(target = zumoThread, args = (5, "zumoTest5", rate, copy.deepcopy(GetDefaultPath(17))))
#t17.start()


##Path 15

#t18 = Thread(target = zumoThread, args = (7, "zumoTest7", rate, copy.deepcopy(GetDefaultPath(15))))
#t18.start()

#t19 = Thread(target = zumoThread, args = (9, "zumoTest9", rate, copy.deepcopy(GetDefaultPath(15))))
#t19.start()


#t20 = Thread(target = zumoThread, args = (10, "zumoTest10", rate, copy.deepcopy(GetDefaultPath(15))))
#t20.start()

##Path16


#t21 = Thread(target = zumoThread, args = (16, "zumoTest16", rate, copy.deepcopy(GetDefaultPath(16))))

#t21.start()

#t22 = Thread(target = zumoThread, args = (13, "zumoTest13", rate, copy.deepcopy(GetDefaultPath(16))))
#t22.start()

#t23 = Thread(target = zumoThread, args = (14, "zumoTest14", rate, copy.deepcopy(GetDefaultPath(16))))
#t23.start()

#t24 = Thread(target = zumoThread, args = (15, "zumoTest15", rate, copy.deepcopy(GetDefaultPath(16))))
#t24.start()

#path19

#t6 = Thread(target = zumoThread, args = (16, "zumoTest16", rate, copy.deepcopy(GetDefaultPath(19))))
#t6.start()

#t25 = Thread(target = zumoThread, args = (17, "zumoTest17", rate, copy.deepcopy(GetDefaultPath(19))))
#t25.start()


#t26 = Thread(target = zumoThread, args = (18, "zumoTest18", rate, copy.deepcopy(GetDefaultPath(19))))
#t26.start()

#path18

#t27 = Thread(target = zumoThread, args = (19, "zumoTest19", rate, copy.deepcopy(GetDefaultPath(18))))
#t27.start()

#t28 = Thread(target = zumoThread, args = (20, "zumoTest20", rate, copy.deepcopy(GetDefaultPath(18))))
#t28.start()

#t29 = Thread(target = zumoThread, args = (21, "zumoTest21", rate, copy.deepcopy(GetDefaultPath(18))))
#t29.start()


#t1 = Thread(target = zumoThread, args = (32, "zumoTest32", 0.05, copy.deepcopy(GetDefaultPath(20))))
#t1.start()
#t2 = Thread(target = zumoThread, args = (33, "zumoTest33", 0.05, copy.deepcopy(GetDefaultPath(20))))
#t2.start()
#t3 = Thread(target = zumoThread, args = (34, "zumoTest34", 0.05, copy.deepcopy(GetDefaultPath(20))))
#t3.start()

t4 = Thread(target = zumoThread, args = (30, "zumoTest30", 0.05, copy.deepcopy(GetDefaultPath(21))))
t4.start()
#t5 = Thread(target = zumoThread, args = (23, "zumoTest23", 0.05, copy.deepcopy(GetDefaultPath(21))))
#t5.start()
#t6 = Thread(target = zumoThread, args = (24, "zumoTest24", 0.05, copy.deepcopy(GetDefaultPath(21))))
#t6.start()
#t7 = Thread(target = zumoThread, args = (25, "zumoTest25", 0.05, copy.deepcopy(GetDefaultPath(21))))
#t7.start()
#t13 = Thread(target = zumoThread, args = (26, "zumoTest26", 0.05, copy.deepcopy(GetDefaultPath(21))))
#t13.start()

#t8 = Thread(target = zumoThread, args = (27, "zumoTest27", 0.05, copy.deepcopy(GetDefaultPath(22))))
#t8.start()
#t9 = Thread(target = zumoThread, args = (16, "zumoTest16", 0.05, copy.deepcopy(GetDefaultPath(22))))
#t9.start()
#t10 = Thread(target = zumoThread, args = (29, "zumoTest29", 0.05, copy.deepcopy(GetDefaultPath(22))))
#t10.start()
#t11 = Thread(target = zumoThread, args = (30, "zumoTest30", 0.05, copy.deepcopy(GetDefaultPath(22))))
#t11.start()
#t12 = Thread(target = zumoThread, args = (31, "zumoTest31", 0.05, copy.deepcopy(GetDefaultPath(22))))
#t12.start()



ani = animation.FuncAnimation(fig, animate, interval=100)
plt.axis('equal')
plt.show()

isRun = False
