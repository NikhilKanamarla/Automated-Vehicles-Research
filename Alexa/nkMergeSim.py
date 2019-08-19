#!/usr/bin/env python
#author Nikhil Kanamarla
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

posx = [0]*36
posy = [0]*36


#index1=9
#name1="zumoTest9"
index2=10
name2="zumoTest10"
#index3=11
#name3="zumoTest11"
#index4=14
#name4="zumoTest14"
#index5=15
#name5="zumoTest15"
#index6=35
#name6="zumoTest35"
#road 1 upper

#posx[index1] = 0.55
#posy[index1] = -0.98
#posx[] = 0.85
#posy[index2] = -0.98
#posx[12] =1.15
#posy[12] = -0.98
#posx[index3] = 1.45
#posy[index3] = -0.98
#posx[12] = 1.75
#posy[12] = -0.98
posx[index2] = 2
posy[index2] = -0.98


#road 2 lower

#posx[index4] = -1.43
#posy[index4] = 0.25
#posx[index5] = -1.7
#posy[index5] = -.144
#posx[index6] = -2.183
#posy[index6] = -.23



#posx[32] = -.2
#posy[32] =1.2
#posx[34] = .3
#posy[34] = 1.2
#posx[9] = .8
#posy[9] = 1.2
#posx[10] = 1.3
#posy[10] = 1.2
#posx[32] = 1.8
#posy[32] = 1.2



def watchout():
    print("it's working")

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
    #ax1.scatter(posx[index1],posy[index1],c='b',s=300)
    ax1.scatter(posx[index2],posy[index2],c='b',s=300)
    #ax1.scatter(posx[index3],posy[index3],c='b',s=300)
    #ax1.scatter(posx[index4],posy[index4],c='r',s=300)
    #ax1.scatter(posx[index5],posy[index5],c='r',s=300)


    #ax1.scatter(posx[index6],posy[index6],c='r',s=300)
    #ax1.scatter(posx[18],posy[18],c='r',s=300)
    #ax1.scatter(posx[19],posy[19],c='r',s=300)
 
    #ax1.scatter(posx[21],posy[21],c='r',s=300)
    #ax1.scatter(posx[22],posy[22],c='r',s=300)
    #ax1.scatter(posx[30],posy[30],c='r',s=300)
    #ax1.scatter(posx[31],posy[31],c='r',s=300)
    #ax1.scatter(posx[32],posy[32],c='r',s=300)

    #chris added these scatters to fully plot all 10 cars
    #ax1.scatter(posx[33],posy[33],c='r',s=300)
    #ax1.scatter(posx[12],posy[12],c='r',s=300)
    #ax1.scatter(posx[28],posy[28],c='r',s=300)
    #ax1.scatter(posx[34],posy[34],c='r',s=300)
    #ax1.scatter(posx[9],posy[9],c='r',s=300)


    ax1.plot(xs0,ys0,'g')
    ax1.plot(xs1,ys1,'b')
    ax1.plot(xc0,yc0,'y')
    ax1.plot(xc1,yc1,'y')
    ax1.plot(xm,ym,'k')
    ax1.set_xlim([3.048,-3.048])
    ax1.set_ylim([1.524,-4.5])

def zumoThread(index, frameName, controlRate, path):
#zumoThread inputs: index (car number) frameName (car name in Vicon), controlRate (???), path (car's path from path.py)
    print("hello")
    global posx
    global posy
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
        
        cur_t = temp_t
        
        rospy.wait_for_service('lf_grad')
        try:
            resp = nf(status, x, y) # from the LineFollowing service, the output res,dx,dy is saved as (ros object?) resp
            res = [resp.res, resp.dx, resp.dy] # turn the ros object resp into a "useable" vector
            status = path.GetSegmentID(x,y) #
            ####################################################
            #stop the car at the end of the path 
            if status==5: #status is the source number of the segment of arc (check mapbuilder.cpp)
                road_speed = 0.0

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

#one zipper merge starting positions
#t1 = Thread(target = zumoThread, args = (index1, name1, 0.05, copy.deepcopy(Path.GetDefaultPath(8))))
#t1.start()

#index2=car 10 and path 40
t1 = Thread(target = zumoThread, args = (index2, name2, 0.05, copy.deepcopy(Path.GetDefaultPath(40))))
t1.start()

#t3 = Thread(target = zumoThread, args = (index3, name3, 0.05, copy.deepcopy(Path.GetDefaultPath(8))))
#t3.start()
#t4 = Thread(target = zumoThread, args = (5, "zumoTest5", 0.05, copy.deepcopy(Path.GetDefaultPath(8))))
#t4.start()
#t5 = Thread(target = zumoThread, args = (12, "zumoTest12", 0.05, copy.deepcopy(Path.GetDefaultPath(8))))
#t5.start()




#t11 = Thread(target = zumoThread, args = (index4, name4, 0.05, copy.deepcopy(Path.GetDefaultPath(0))))
#t11.start()
#t12 = Thread(target = zumoThread, args = (index5, name5, 0.05, copy.deepcopy(Path.GetDefaultPath(0))))
#t12.start()
#t13 = Thread(target = zumoThread, args = (index6, name6, 0.05, copy.deepcopy(Path.GetDefaultPath(0))))
#t13.start()

#t15 = Thread(target = zumoThread, args = (9, "zumoTest9", 0.05, copy.deepcopy(Path.GetDefaultPath(3))))
#t15.start()
#t16 = Thread(target = zumoThread, args = (10, "zumoTest10", 0.05, copy.deepcopy(Path.GetDefaultPath(3))))
#t16.start()
#t17 = Thread(target = zumoThread, args = (34, "zumoTest34", 0.05, copy.deepcopy(Path.GetDefaultPath(3))))
#t17.start()
#t18 = Thread(target = zumoThread, args = (32, "zumoTest32", 0.05, copy.deepcopy(Path.GetDefaultPath(3))))
#t18.start()
#t19 = Thread(target = zumoThread, args = (10, "zumoTest10", 0.05, copy.deepcopy(Path.GetDefaultPath(3))))
#t19.start()

ani = animation.FuncAnimation(fig, animate, interval=100)
plt.axis('equal')
plt.show()

isRun = False
