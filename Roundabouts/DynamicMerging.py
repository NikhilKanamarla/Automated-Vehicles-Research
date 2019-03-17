#!/usr/bin/env python
#todo: implement Vicon tracking system
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
TestActive = True
style.use('fivethirtyeight')

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)

#t = 0
xs0,ys0,xs1,ys1,xc0,yc0,xc1,yc1,xm,ym = BuildPath(0.05)

posx = [0]*36
posy = [0]*36


index1=20
name1="zumoTest20"
index2=29
name2="zumoTest29"
index3=17
name3="zumoTest17"
index4=23
name4="zumoTest23"
index5=25
name5="zumoTest25"
index6=4
name6="zumoTest4"
index7=10
name7="zumoTest10"
index8=24
name8="zumotest24"
index9 = 11
name9 = "zumotest11"
index10 = 26
name10 = "zumotest26"
index11 = 30
name11 = "zumotest30"
index12 = 35
name12 = "zumotest35"

inds = [name1,name2,name3, name4, name5, name6, name7, name8, name9, name10, name11, name12]

inds2 = [index1, index2, index3, index4, index5, index6, index7, index8, index9, index10, index11, index12]

#road 1 upper start
posx[index1] = 1.32
posy[index1] = 0.756802
posx[index2] = 1.0
posy[index2] = 0.76406

#road 2 lower start 
posx[index3] = 0.7
posy[index3] = 0.76406
posx[index4] = 0.4
posy[index4] = 0.76406

#right roundabouts

posx[index5] = -2.64291
posy[index5] = 0.30423
posx[index6] = -2.64291
posy[index6] = 0.30423
posx[index7] = -2.51
posy[index7] = 0.038
posx[index8] = -2.5
posy[index8] = 0.73

#extra cars
posx[index9] = 0
posy[index9] = 0.756802
posx[index10] = -0.381052
posy[index10] = 0.756802
posx[index11] = 0.5
posy[index11] = 1.22389
posx[index12] = 1 
posy[index12] = 1.22389



def talker(index,x,y,drx,dry,sp):
    global pub
    pub.publish(str(index)+","+str(x)+","+str(y)+","+str(drx)+","+str(dry)+","+str(sp))

def RT_control(to,tmi,xi,xf,v,vf):
    A_mtx = np.matrix([[to**3/6,to**2/2,to,1],[to**2/2,to,1,0],[tmi**3/6,tmi**2/2,tmi,1],[tmi**2/2,tmi,1,0]])
    Y_mtx = np.matrix([[xi],[v],[xf],[vf]])

    A_aux = np.transpose(A_mtx)*A_mtx
    X_mtx = pinv(A_aux)*np.transpose(A_mtx)*Y_mtx

    return X_mtx

def getViconPos(frameName):
    global tfl 

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

def animate(i):
    global xs0#long road
    global ys0#Long road
    global xs1
    global ys1

    ax1.clear()

	#the dot at 0,0 is caused by having extra indexes here that don't corresspond to an actual car in the sim.
    ax1.scatter(posx[index1],posy[index1],c='r',s=300)
    ax1.scatter(posx[index2],posy[index2],c='b',s=300)
    ax1.scatter(posx[index3],posy[index3],c='g',s=300)
    ax1.scatter(posx[index4],posy[index4],c='y',s=300)
   # ax1.scatter(posx[index5],posy[index5],c='r',s=300)
    ax1.scatter(posx[index6],posy[index6],c='b',s=300)
    ax1.scatter(posx[index7],posy[index7],c='g',s=300)
    ax1.scatter(posx[index8],posy[index8],c='y',s=300)
    #ax1.scatter(posx[index9],posy[index9],c='r',s=300)
    #ax1.scatter(posx[index10],posy[index10],c='b',s=300)
    #ax1.scatter(posx[index11],posy[index11],c='g',s=300)
    #ax1.scatter(posx[index12],posy[index12],c='y',s=300)

    ax1.plot(xs0,ys0,'g')
    ax1.plot(xs1,ys1,'b')
    ax1.plot(xc0,yc0,'y')
    ax1.plot(xc1,yc1,'y')
    ax1.plot(xm,ym,'k')
    ax1.set_xlim([3.048,-3.048])
    ax1.set_ylim([1.524,-4.5])


def zumoThread(index, frameName, controlRate, path):
#zumoThread inputs: index (car number) frameName (car name in Vicon), controlRate (???), path (car's path from path.py)
    global posx
    global posy
    global TestActive	
    x = posx[index]
    y = posy[index]
    status = path.GetSegmentIDNoCheck()#return the Path segment number 
    
    road_speed = 0.2 #speed limit on the road unless intervened by from the controller
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
        """
        if TestActive:
            try:
                x,y,t=getViconPos(frameName)
            except: 
                if DEBUG:
                    print("VICON ERROR, Vicon cannot pass positions of vehicle back")
                
            posx[index]=x
            posy[index]=y
            streetindex=findMe(index,headwayManager.nodes[paths[j]],k[j],1)
            #Snap vehicle position to node
            try:
                x=headwayManager.nodes[paths[j]][streetindex][0]
                y=headwayManager.nodes[paths[j]][streetindex][1]
            except:
                if DEBUG:
                    print("Projected x and y cannot be found from headway")
        """         
        cur_t = temp_t
        
        rospy.wait_for_service('lf_grad')
        try:
            resp = nf(status, x, y) # from the LineFollowing service, the output res,dx,dy is saved as (ros object?) resp
            res = [resp.res, resp.dx, resp.dy] # turn the ros object resp into a "useable" vector
            status = path.GetSegmentID(x,y) # 

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
                    print(isControlled)
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

'''
if TestActive:
    for i in range(12):
        x,y,t=getViconPos(inds[i])
        posx[inds2[i]]=x
        posy[inds2[i]]=y
'''	
rospy.init_node('zumo_go', anonymous=True)#zumo_go is a node
global tfl
tfl = tf.TransformListener()

pub = rospy.Publisher('/ZumoRefs', String, queue_size=1000)#ZumoRefs is a topic name 

#one zipper merge starting positions
t1 = Thread(target = zumoThread, args = (index1, name1, 0.05, copy.deepcopy(Path.GetDefaultPath(39))))
t1.start()
t2 = Thread(target = zumoThread, args = (index2, name2, 0.05, copy.deepcopy(Path.GetDefaultPath(39))))
t2.start()
t3 = Thread(target = zumoThread, args = (index3, name3, 0.05, copy.deepcopy(Path.GetDefaultPath(39))))
t3.start()
t4 = Thread(target = zumoThread, args = (index4, name4, 0.05, copy.deepcopy(Path.GetDefaultPath(39))))
t4.start()


#t5 = Thread(target = zumoThread, args = (index5, name5, 0.05, copy.deepcopy(Path.GetDefaultPath(45))))
#t5.start()
t5 = Thread(target = zumoThread, args = (index6, name6, 0.05, copy.deepcopy(Path.GetDefaultPath(48))))
t5.start()
t6 = Thread(target = zumoThread, args = (index7, name7, 0.05, copy.deepcopy(Path.GetDefaultPath(48))))
t6.start()
t7 = Thread(target = zumoThread, args = (index8, name8, 0.05, copy.deepcopy(Path.GetDefaultPath(48))))
t7.start()


#t8 = Thread(target = zumoThread, args = (index9, name9, 0.05, copy.deepcopy(Path.GetDefaultPath(39))))
#t8.start()
#t9 = Thread(target = zumoThread, args = (index10, index10, 0.05, copy.deepcopy(Path.GetDefaultPath(39))))
#t9.start()
#t10 = Thread(target = zumoThread, args = (index11, index11, 0.05, copy.deepcopy(Path.GetDefaultPath(44))))
#t10.start()
#t11 = Thread(target = zumoThread, args = (index12, index12, 0.05, copy.deepcopy(Path.GetDefaultPath(44))))
#t11.start()

ani = animation.FuncAnimation(fig, animate, interval=100)
plt.axis('equal')
plt.show()

isRun = False
