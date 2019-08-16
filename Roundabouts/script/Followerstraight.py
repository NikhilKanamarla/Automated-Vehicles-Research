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
#from straight_controller.dir import Dir
from control import StraightController as SC
#import execnet


isRun = True

style.use('fivethirtyeight')

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)

#t = 0
xs0,ys0,xs1,ys1,xc0,yc0,xc1,yc1,xm,ym = BuildPath(0.05)

posx = [0]*36
posy = [0]*36

#For path index 13 Starting at S68
#initial position for the leader 
posx[9] = 2.4
posy[9] = -1.15
posx[34] = 2.6
posy[34] = -1.15

#def call_python_version(Version, Module, Function, ArgumentList):
#    gw      = execnet.makegateway("popen//python=python%s" % Version)
#    channel = gw.remote_exec("""
#        from %s import %s as the_function
#        channel.send(the_function(*channel.receive()))
#    """ % (Module, Function))
#    channel.send(ArgumentList)
#    return channel.receive()

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



def talker(index,x,y,drx,dry,sp):
    global pub
    #print("Publishing: " + str(index)+","+str(x)+","+str(y)+","+str(drx)+","+str(dry)+","+str(sp))
    pub.publish(str(index)+","+str(x)+","+str(y)+","+str(drx)+","+str(dry)+","+str(sp))



def animate(i):
    #global t
    global xs0
    global ys0
    global xs1
    global ys1

    ax1.clear()

    ax1.scatter(posx[9],posy[9],c='b',s=210)
    ax1.scatter(posx[34],posy[34],c='r',s=210)


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
    global posx_old
    global control
    xL,yL,tL=getViconPos(9)
    posx[9]=xL
    posy[9]=yL
    #xL=posx[9]
    #yL=posy[9]
    xF = posx[index]
    posxL_old=posx[9]
    posxF_old=posx[index]
    yF = posy[index]
    
	

    status = path.GetSegmentIDNoCheck()
    road_speed = 0.2
    speed_limit = .2*1.5
    road_length = 1500.0
    speed = road_speed #*1.145 #linear scaling factor 1.145 to achieve 0.30 actual

    nf = rospy.ServiceProxy('lf_grad', LineFollowing)
    nf_m = rospy.ServiceProxy('lf_merge', MergeControl)

    cur_t = time.time()

    dxF = -0.0
    dyF = 0.0
    dxL=-.2;
    isControlled = False
    abcd = np.matrix([[0],[0],[0],[0]])
    tinit = 0
    controlInfoPre = (None,None)
    while isRun:
		xL,yL,tL=getViconPos(9)
		temp_t = time.time()
		posx[9]=xL
		posy[9]=yL
                dxL=(posx[9]-posxL_old)/(temp_t-cur_t)
                #print("PosX Current: " +str(posx[9]) + " vs Old: " +str(posxL_old))
                dxL=round(dxL,2)
		#xL=xL+dxL*(temp_t-cur_t);
                posxL_old=posx[9]
                posx[9]=xL
                headway = float(-posx[9]+posx[index])-.21
                observation = [headway*25/road_length,abs(dxF/speed_limit),abs(dxL/speed_limit)]
                #a=10/road_length;
                #b=10.0/15.0;
                #c=10/15.0;
                #observationTEST = (a,b,c)
                print("Observation: " +str(observation))
                #print(observationTEST)
                axF = control.get_action(observation)/25
                print("axF: " + str(axF))
                #axFTEST = control.get_action(observationTEST)
                #print("axFTEST: " + str(axFTEST))
                xF = xF - .5*axF*(temp_t-cur_t)*(temp_t-cur_t) +dxF*(temp_t-cur_t)
                dxF = dxF-axF*(temp_t-cur_t)
                #print("time: " + str(temp_t-cur_t))
                #print("dxF: " + str(dxF))
		 #dxF*(temp_t-cur_t)
		#y = y
		posx[index] = xF
		#posy[index] = y
                xF=float(xF)
                xF=round(xF,2)

                yF=float(yF)
                yF=round(yF,2)

                dxF=float(dxF)
                dxF=round(dxF,2)

                dyF=float(dyF)
                dyF=round(dyF,2)
                
		talker(index,xF,yF,dxF,dyF,speed)
		cur_t = temp_t

		time.sleep(controlRate)


#rospy.init_node('zumo_go', anonymous=True)


pub = rospy.Publisher('/ZumoRefs', String, queue_size=1000)
def main():
    global control
    control=SC("weight_4.pkl")
    #control = call_python_version("3.5","control","__init__","weight_0.pkl")
    global tfl
    tfl = tf.TransformListener()
    rospy.init_node('dummy')
    t1 = Thread(target = zumoThread, args = (34, "zumoTest34", rate, copy.deepcopy(GetDefaultPath(26))))
    t1.start()

if __name__ == '__main__':

    main()



##Path 13



ani = animation.FuncAnimation(fig, animate, interval=100)
plt.axis('equal')
plt.show()


