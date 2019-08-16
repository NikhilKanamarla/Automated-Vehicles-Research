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
import Headway2
from pathplot import pathplotbuilder
from numpy.linalg import inv,pinv
from threading import Thread, Lock
import threading
from geometry_msgs.msg import TransformStamped
from line_following.srv import *
from socketIO_client import SocketIO, LoggingNamespace
#Thread Variables
rate = .1 #.05
num=0
time_c=0

#import Path
import copy
from trafficSignControls import trafficYield
from trafficSignControls import inZone
from transformation import transformCoords
from transformation import measureCircle
from idm_controller import idm
from collections import deque
from control import StraightController as RLController
import random

#KD Tree for HW
from scipy.spatial import cKDTree

DEBUG = False
isRun = True
SAVE_DATA = True

style.use('fivethirtyeight')

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)
###Yield Signs
##Yielding Zones
xmaxA=3.048-4.52
xminA=3.048-4.83
ymaxA=1.524-0.3
yminA=1.524-0.63
xmaxB=3.048-4.49
xminB=3.048-5.0
ymaxB=1.524-1.87#-1.83
yminB=1.524-2.18
##Yield to Zones
x_minA=3.048-5.25
x_maxA=3.048-4.52
y_minA=1.524-0.93
y_maxA=1.524-0.59
x_minB=3.048-5.72
x_maxB=-1.956
y_minB=1.524-1.83
y_maxB=1.524-1.59

###"Stop Signs"
#Defines range of values that the stop sign randomly starts at
stopAmax = 3.048-1.37
stopAmin = 3.048-2.29
stopBmax = 1.524-3.76
stopBmin = 1.524-4.65
#Defines road width
stopx_maxB = 3.048-4.93
stopx_minB = 3.048-5.07
stopy_maxA = 1.524-0.23
stopy_minA = 1.524-0.54
#Initialize stop sign
stopB=random.uniform(stopBmin,stopBmax)
stopA=random.uniform(stopAmin,stopAmax)

expStarted = False
experimentDone = True
leadInStopA = False
leadInStopB = False
stopOn = True

stopDistance = .05 #Distance of car to start of stop sign (max)
carDistance = .25 #Distance between cars when stopped (max)

#Platoons
a_min = 2
a_max = 8
b_min = 2
b_max = 5
#a_min = 1
#a_max = 2
#b_min = 2
#b_max = 5
n_A = random.randrange(a_min,a_max,1)
n_B = random.randrange(b_min,b_max,1)
flush_a = True
flush_b = True
platoonAReady = False
platoonBReady = False

#Indices
ind1=7
ind2=34
ind3=29
ind4=32
ind5=35

ind6=33
ind7=4
ind8=10
ind9=14
ind10=17
ind11=23
ind12=24
ind13=26

#RouteA
indiciesB=[ind1,ind2,ind3,ind4,ind5]
indiciesA=[ind6,ind7,ind8,ind9,ind10,ind11,ind12,ind13]
#indiciesA=[ind6,ind7]

#Path Indicies and vehicle locations
paths=[29,30]#[29,32,31,30,33]
#initialization of the pathLocations for column of path number and rows of vehicles with 0 
pathLocations=[[0,0] for _ in range(36)]

#t = 0 Path plot creation
xs0,ys0,xs1,ys1,xc0,yc0,xc1,yc1,xm,ym = BuildPath(0.05)
#x,y=pathplotbuilder(

#State Variables
scalingFactor =25*1.357
nVehicles = 36
posx = [0]*36
posy = [0]*36
velx = [0]*36
vely = [0]*36
roundLength = [[100,100] for _ in range(36)]
queueA = [[-1,-1] for _ in range(36)]
queueB = [[-1,-1] for _ in range(36)]
inQueueA = 0
inQueueB = 0
absPos = [-1]*36

#Queue Parameters
queueArcLengthA=3.91*(1.9661-1.5996) #A106 Arc length leading into the roundabout
queueArcLengthB=.67*(3.1416-2.1949) #A102
QxRefA=3.048-5.32
QyRefA=1.524-0.61
QxCenterA=-1.282
QyCenterA=-.686
QxRefB=3.048-4.72
QyRefB=1.524-1.66
QxCenterB=-.762
QyCenterB=-2.696
xvalueA= 3.048-3.92#For measuring straightaway of upper queue
yvalueB= 1.524-2.21#For measuring straightaway of lower queue

#Roundabout Parameters
xRefR=3.048-4.76
yRefR=1.524-.65
xCenterR=-2.0232
yCenterR=.354

#RL Deques
eAD=deque() #stores all RL vehicles in experiment that entered from lower queue
eBD=deque() #stores all RL vehicles in experiment that entered from upper queue


prob = 0#probability of a vehicle being an RL vehicle
random.seed(time.time())
NN_a1 = deque(maxlen = 1)
NN_a2 = deque(maxlen = 1)
NN_a1.append(0)
NN_a2.append(0)

#Normalization Factors
merge_0_norm=74.32
merge_1_norm=86.57
queue_0_norm=16.0
queue_1_norm=19.0
total_scenario_length=443.25
max_speed=8
roundabout_length=130.98

#Absolute Position Reference Points and Distances
roundaboutLength=.61*2*math.pi #Length of the Roundabout (all the way around)
arcALength=3.91*(1.9661-1.5996) #length of the arc of the lower entrance
arcEBLength=.67*.9467 #Length of the arc of the upper exit
arcBLength=.67*(3.1416-2.1949) #Length of the arc of the upper entrance
arcEALength=.56*(3.013-1.5708) #length of the arc of the lower exit
straightALength=merge_1_norm/scalingFactor-arcALength #Length of the straightaway of the lower entrance
straightBLength=merge_0_norm/scalingFactor-arcBLength #length of straightaway of the upper entrance
straightEBLength=74.32/scalingFactor-arcEBLength #length of straightaway exit upper
xExitA=3.048-3.92
QxRefA2=3.048-3.982
QyRefA2=1.524-.31
ExRefB=3.048-5.44
EyRefB=1.524-1.66
yExitB=1.524-2.21
QxRefB2=3.048-5.0
QyRefB2=1.524-2.21
ExRefA=3.048-4.48
EyRefA=1.524-1.25
ExCenterA=-.872
EyCenterA=.204
ExCenterB=-2.782
EyCenterB=-.686
xStartA=QxRefA2+straightALength
yStartB=QyRefB2-straightBLength
xStartAUC=xStartA
yStartBUC=yStartB


#Upper Loop
#source path_index=41
posx[ind1] = -0.232
posy[ind1] =  -0.89
#source path_index=34
posx[ind2] = -0.232
posy[ind2] = -0.59
posx[ind3] = -0.232
posy[ind3] = -0.29
posx[ind4] = -0.232
posy[ind4] = 0.01
posx[ind5] = -0.232
posy[ind5] = 0.21     




#Lower Loop
#source path_index=43
posx[ind6] = 1.55
posy[ind6] = 0.65
#source path_index=42
posx[ind7] = 1.1
posy[ind7] = 0.754
posx[ind8] = 0.8
posy[ind8] = 0.754
#source path_index=39
posx[ind9] = 0.5
posy[ind9] = 0.754
posx[ind10] = 0.2
posy[ind10] = 0.754
posx[ind11] = -0.1
posy[ind11] = 0.754
posx[ind12] = -0.4
posy[ind12] = 0.754
posx[ind13] = -0.7
posy[ind13] = 0.754

#ViconData
viconX=[0]*36
viconY=[0]*36
viconT=[0]*36

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

def viconDataMaster():
    global viconX
    global viconY
    global viconT
    global tfl
    tfl = tf.TransformListener()
    for i in range(nVehicles):
        try:
            name="zumoTest"+str(i)
            x,y,theta=getViconPos(name)
            viconX[i]=x
            viconY[i]=y
            viconT[i]=theta
        except:
            viconX[i]=0
            viconY[i]=0
            viconT[i]=0


def getRLInfo(index, pathIndex):
    #t1 = time.time()
    #rl_pos - get the position of this agent
    rl_pos = absPos[index]*scalingFactor/total_scenario_length
    #rl_pos2- get the position along the roundabout 
    rl_pos_2 = absPos[index]*scalingFactor/roundabout_length
    if rl_pos_2 > 1: #if we're outside the roundabout zone
        rl_pos_2 = 0
    #rl_vel - calculate speed from velocity
    rl_vel = math.sqrt( (velx[index]*velx[index]) + (vely[index]*vely[index]) ) * scalingFactor / max_speed
    #get headway and tailway
    column = [item[paths.index(pathIndex)] for item in pathLocations]
    indh,hw,indt,tw=getHeadway(index,column,len(headwayManager.nodes[pathIndex]))

    tailway_vel = 0
    tailway_dist = 0
    headway_vel = 0
    headway_dist = 0
    #get tailway vars if they exist
    if (indt != index) and (absPos[indt] > 0):
        #tailway_vel - velocity/max_speed of follower, or 0 if no follower
        tailway_dist = tw*scalingFactor/total_scenario_length
        #tailway_dists - tailway/total length, or 0 if no follower
        tailway_vel = math.sqrt( (velx[indt]*velx[indt]) + (vely[indt]*vely[indt]) ) * scalingFactor / max_speed
        if tailway_dist > 1:
            tailway_dist = 0
            tailway_vel = 0

    #get headyway vars if they exist
    if (indh != index) and (absPos[indh]*scalingFactor < total_scenario_length):
        #headway_vel - velocity/max speed, 0 if no follower
        headway_vel = math.sqrt( (velx[indh]*velx[indh]) + (vely[indh]*vely[indh]) ) * scalingFactor / max_speed
        #headway_dists - headway/total length, or 0 if no follower
        headway_dist = hw*scalingFactor/total_scenario_length
        if headway_dist > 1:
            headway_dist = 0
            headway_vel = 0
    
    len_inflow_B = (n_B-1) / 7.0
    len_inflow_A = (n_A-1) / 7.0

    return [rl_pos, rl_pos_2, rl_vel, tailway_vel, tailway_dist, headway_vel, headway_dist]#, len_inflow_B, len_inflow_A]

def getIDMSpeed(index,speed,delT,road_speed,pathIndex):
    column = [item[paths.index(pathIndex)] for item in pathLocations]
    ind,hw,indt,tw=getHeadway(index,column,len(headwayManager.nodes[pathIndex]))
    sn=(hw)
    vmax=road_speed
    v0 = math.sqrt((velx[index]*velx[index])+(vely[index]*vely[index]))
    v1 = math.sqrt((velx[ind]*velx[ind])+(vely[ind]*vely[ind]))
    a=idm(sn, v0, v1, vmax)
    speed = speed+a*delT
    speed = min(max(0,speed),road_speed)

    return(speed)

def getRLAction(ind1, ind2, pathIndex1, pathIndex2):
    global isRun
    #t1 = time.time()

    temp0Array=copy.deepcopy(queueB)
    temp0Array=sorted(temp0Array,key=lambda x:x[0])
    merge_dists_0=[]
    merge_0_vel=[]
    for i in range(len(temp0Array)):
        if temp0Array[i][0]>=0:
           temp=temp0Array[i][0]*scalingFactor/queue_0_norm
           if temp<=1:
               merge_dists_0.append(temp)
               merge_0_vel.append(temp0Array[i][1]*scalingFactor/max_speed)
           else:
               merge_dists_0.append(0)
               merge_0_vel.append(0)
        if len(merge_dists_0) == 6:
            break
    merge_dists_0.extend([0]*(6-len(merge_dists_0)))
    merge_0_vel.extend([0]*(6-len(merge_0_vel)))
    #t2 = time.time()
    #merge_dists_1 upper queue distances. Sort by increasing distance
    #merge_1_vel velocity/max speed for vehicles in lower queue
    temp1Array=copy.deepcopy(queueA)
    temp1Array=sorted(temp1Array,key=lambda x:x[0])
 
    merge_dists_1=[]
    merge_1_vel=[]
    for i in range(len(temp1Array)):
        if temp1Array[i][0]>=0:
           temp=temp1Array[i][0]*scalingFactor/queue_1_norm
           if temp<=1:
               merge_dists_1.append(temp)
               merge_1_vel.append(temp1Array[i][1]*scalingFactor/max_speed)
           else:
               merge_dists_1.append(0)
               merge_1_vel.append(0)
        if len(merge_dists_1) == 6:
            break
    merge_dists_1.extend([0]*(6-len(merge_dists_1)))
    merge_1_vel.extend([0]*(6-len(merge_1_vel)))
    #t3 = time.time()

    #queue_0 upper queue length/11
    queue_0=inQueueB/queue_0_norm
    #queue_1 lower queue length/14
    queue_1=inQueueA/queue_1_norm
    #roundabout full 26x2 array, 0th column is abs pos/roundabout length 1st column is veloctiy/max speed for vehicles in roundabout. Sort by increasing absolute distance. Pad in zeros for remaining
    tempRoundArray=copy.deepcopy(roundLength)
    tempRoundArray=sorted(tempRoundArray,key=lambda x:x[0])

    roundabout_full=tempRoundArray[:26][:26]

    for i in range(26):
        if roundabout_full[i][0]==100:
           roundabout_full[i][0]=0
           roundabout_full[i][1]=0
        else:
           roundabout_full[i][0]=roundabout_full[i][0]*scalingFactor/roundabout_length
           roundabout_full[i][1]=roundabout_full[i][1]*scalingFactor/max_speed
    #I hate myself
    #t4 = time.time()
    roundabout_fullFinal=[]
    for i in range(26): #For each row
        for j in range(2): #For each column
            roundabout_fullFinal.append(roundabout_full[i][j])

    #t10 = time.time()

    #check if we are in A or B
    if ind1 != 0:
        rl_info1 = getRLInfo(ind1, pathIndex1)
    else:
	    rl_info1 = [0]*7

    if ind2 != 0:
        rl_info2 = getRLInfo(ind2, pathIndex2)
    else:
        rl_info2 = [0]*7
    #t5 = time.time()

    len_inflow_B = (n_B-1) / 7.0
    len_inflow_A = (n_A-1) / 7.0

    obs=rl_info1+rl_info2+merge_dists_0+merge_0_vel+merge_dists_1+merge_1_vel+[queue_0]+[queue_1]+roundabout_fullFinal + [len_inflow_B] + [len_inflow_A]
    
    try:
    	a,b=control.get_action(np.asarray(obs))#/scalingFactor#Get Acceleration
    except:
        print(n_A,n_B)


    #t6 = time.time()



    return(a,obs)

def NNMaster():

    NN_a1.append(0)
    NN_a2.append(0)
    while isRun:
        #get RL vehicle information
        ind1 = 0
        ind2 = 0
        pathindex1 = 0
        pathindex2 = 0

        if eAD and len(eAD) > 0:
            ind1 = eAD[0]
            pathindex1 = pathIndexTable[ind1]
        if eBD and len(eBD) > 0:
            ind2 = eBD[0]
            pathindex2 = pathIndexTable[ind2]
        try:
        #get actions for both vehicles
            a,obs = getRLAction(ind2, ind1, pathindex2, pathindex1)
            #print (obs , a)
        #push to deques
            NN_a1.append(a[1]/scalingFactor)
            NN_a2.append(a[0]/scalingFactor)
        except:
            b=0

def headwayMaster(paths):
    k=[0]*len(paths)
    for i in range(len(paths)): #Makes a KD tree for each path
        k[i]=cKDTree(headwayManager.nodes[paths[i]],leafsize=40)

    for j in range(len(paths)):
        pathLocations[0][j] = None
    while isRun:
        for i in range(35): #For each vehicle
            for j in range(len(paths)): #For each Path
                pathLocations[i+1][j]=findMe(i+1,headwayManager.nodes[paths[j]],k[j]) #stores each vehicles location along a path

def getHeadway(index,vehicleNodes,pathMax,nodeDistance=.01):

    #Function returns headway and tailway of a vehicle of a given index as well as the indicies of those vehicles
    #vehicleNodes is a column of the node location along the path of each vehicle
    #pathMax is the number of nodes in the path
    #If the function fails to find itself or gets some other value, it returns yourself. This will most normally happen due to a large time step causing a vehicle to "fall off the road"
    try:
        tempArray = copy.deepcopy(vehicleNodes) #Vehicle Nodes are the positions of each vehicle along their path 
        tempArray = sorted(tempArray)
        sortedArray = [x for x in tempArray if x is not None]
        me=vehicleNodes[index]
        newInd=sortedArray.index(me)
    
        if len(sortedArray)==1:
            headway = pathMax*nodeDistance
            indH =  index
            tailway = pathMax*nodeDistance
            indT = index
        elif newInd == 0: #if you're first along the path
            headway = (sortedArray[newInd+1]-me)*nodeDistance
            indH =  vehicleNodes.index(sortedArray[newInd+1])
            tailway = pathMax-(-me+sortedArray[-1])*nodeDistance#(pathMax - (me-sortedArray[len(sortedArray)]))*nodeDistance
            indT = vehicleNodes.index(sortedArray[-1])
        elif newInd == len(sortedArray)-1: #if you're last along the path
            headway = (pathMax-(-sortedArray[0]+me))*nodeDistance
            indH =  vehicleNodes.index(sortedArray[0])
            tailway = (me-sortedArray[newInd-1])*nodeDistance
            indT = vehicleNodes.index(sortedArray[newInd-1])
        else: #you're in the middle
            headway = (sortedArray[newInd+1]-me)*nodeDistance
            indH =  vehicleNodes.index(sortedArray[newInd+1])
            tailway = (me-sortedArray[newInd-1])*nodeDistance
            indT = vehicleNodes.index(sortedArray[newInd-1])
        return indH,headway,indT,tailway
    except:
        print("WARNING: Error occured in headway for vehicle " + str(index))
        return index,pathMax*nodeDistance,index,pathMax*nodeDistance

def findMe(index,nodes,k,streetSize=.1):
   dist,streetIndex = k.query([posx[index],posy[index]])
   if dist<=streetSize:
      return streetIndex
   else:
      return None

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



    ax1.scatter(posx[ind1],posy[ind1],c='g',s=200)
    ax1.scatter(posx[ind2],posy[ind2],c='b',s=200)
    ax1.scatter(posx[ind3],posy[ind3],c='b',s=200)
    ax1.scatter(posx[ind4],posy[ind4],c='b',s=200)
    ax1.scatter(posx[ind5],posy[ind5],c='b',s=200)
    
    ax1.scatter(posx[ind6],posy[ind6],c='g',s=200)
    ax1.scatter(posx[ind7],posy[ind7],c='r',s=200)
    ax1.scatter(posx[ind8],posy[ind8],c='r',s=200)
    ax1.scatter(posx[ind9],posy[ind9],c='r',s=200)
    ax1.scatter(posx[ind10],posy[ind10],c='r',s=200)
    ax1.scatter(posx[ind11],posy[ind11],c='r',s=200)
    ax1.scatter(posx[ind12],posy[ind12],c='r',s=200)
    ax1.scatter(posx[ind13],posy[ind13],c='r',s=200)

    ax1.scatter(stopA,(stopy_maxA+stopy_minA)/2,c='y',s=100)
    ax1.scatter((stopx_maxB+stopx_minB)/2,stopB,c='y',s=100)

    #ax1.plot(xs0,ys0,'g')
    #ax1.plot(xs1,ys1,'b')
    #ax1.plot(xc0,yc0,'y')
    #ax1.plot(xc1,yc1,'y')
    ax1.plot(path1X,path1Y,'b')
    #ax1.plot(path2X,path2Y,'g')
    #ax1.plot(path3X,path3Y,'r')
    ax1.plot(path4X,path4Y,'y')

    if len(eAD) > 0:
    	ax1.scatter(posx[eAD[0]],posy[eAD[0]],c='k',s=250)
    if len(eBD) > 0:
    	ax1.scatter(posx[eBD[0]],posy[eBD[0]],c='k',s=250)
    ax1.set_xlim([3.048,-3.548])
    ax1.set_ylim([1.524,-5])
    plt.axis('equal')


def csvout():
    #time, q1 size, q2 size, q1 state, q2 state, roundabout state
    DATA_OUT = ''
    header = 't,r1_i,r2_i,q1,q2,'
    pos = ''
    q1p = ''
    q1v = ''
    q2p = ''
    q2v = ''
    rbp = ''
    rbv = ''
    vXL = ''
    vYL = ''
    vTL = ''
    for i in range(nVehicles):
        pos += 'pos-' + str(i) + ','
        q1p += 'qA-'+str(i)+'-pos,'
        q1v += 'qA-'+str(i)+'-vel,'
        q2p += 'qB-'+str(i)+'-pos,'
        q2v += 'qB-'+str(i)+'-vel,'
        rbp += 'rb-'+str(i)+'-pos,'
        rbv += 'rb-'+str(i)+'-vel,'
        #Vicon Data Collection
        vXL += 'viconX-'+str(i)+','
        vYL += 'viconY-'+str(i)+','
        vTL += 'viconT-'+str(i)+','
    header += pos + q1p + q1v + q2p + q2v + rbp + rbv + vXL + vYL + vTL+ 'flush' + '\n'
    #open a unique file for this data
    currentDate = time.strftime('%m_%d_%y__%H_%M_%S')
    datafile = open("/home/themainframe/catkin_ws/src/line_following/data/UCB/roundaboutRL%s.csv" %currentDate,"w")
    #write the header
    datafile.write(header)
    datafile.close()
    #grab and write data
    while isRun:
        viconDataMaster()
        wtime = time.time()
        rl1 = -1
        rl2 = -1
        wnq1 = inQueueA
        wnq2 = inQueueB
        wpos = copy.deepcopy(absPos)
        wq1 = copy.deepcopy(queueA)
        wq2 = copy.deepcopy(queueB)
        wrb = copy.deepcopy(roundLength)
        vx=copy.deepcopy(viconX)
        vy=copy.deepcopy(viconY)
        vt=copy.deepcopy(viconT)
        isflushing = 0
        if (flush_a or flush_b):
            isflushing = 1

	if len(eAD) > 0:
		rl1 = eAD[0]
	if len(eBD) > 0:
		rl2 = eBD[0]


        data = str(wtime) + ',' + str(rl1) + ',' + str(rl2) + ',' + str(wnq1) + ',' + str(wnq2) + ','

        pos = ''
        q1p = ''
        q1v = ''
        q2p = ''
        q2v = ''
        rbp = ''
        rbv = ''
        vxD = ''
        vyD = ''
        vtD = ''
        for i in range(nVehicles):
            pos += str(wpos[i]) + ','
            q1p += str(wq1[i][0]) + ','
            q1v += str(wq1[i][1]) + ','
            q2p += str(wq2[i][0]) + ','
            q2v += str(wq2[i][1]) + ','
            rbp += str(wrb[i][0]) + ','
            rbv += str(wrb[i][1]) + ','
            vxD +=str(vx[i])+','
            vyD +=str(vy[i])+','
            vtD +=str(vt[i])+','
        data += pos + q1p + q1v + q2p + q2v + rbp + rbv + vxD + vyD + vtD + str(isflushing) + '\n'

        DATA_OUT += data


        #datafile.write(data)
        #end of loop
    
    datafile = open("/home/themainframe/catkin_ws/src/line_following/data/UCB/roundaboutRL%s.csv" %currentDate,"a")
    datafile.write(DATA_OUT)
    datafile.close()


timerA=time.time()
timerB=time.time()
def zumoThread(index, frameName, controlRate, path,pathIndex):
    stopped = False
    global isRun
    global posx
    global posy
    global inQueueA
    global inQueueB
    global queueA
    global queueB
    global roundLength
    global roundabout_length
    global eBD
    global eAD
    global timerA
    global timerB
    global expStarted
    global n_A
    global n_B
    global stopOn
    global experimentDone 
    global leadInStopA 
    global leadInStopB 
    global stopDistance 
    global carDistance 
    global a_min 
    global a_max 
    global b_min
    global b_max 
    global flush_a 
    global flush_b 
    global platoonAReady 
    global platoonBReady 
    global stopA
    global stopB
    global num
    global time_c
    
    RL = False
    if index == ind1 or index == ind6:
        localP=1
        #localP=0
    else:
        localP=0
    

    enteredExp = False
    x = posx[index]
    y = posy[index]
    status = path.GetSegmentIDNoCheck()
    road_speed = .21#0.236
    speed = 0 #Starting speed
    noInQueueA=True
    noInQueueB=True
    notFirst = False
    nf = rospy.ServiceProxy('lf_grad', LineFollowing, persistent=False)
    pathIndexTable[index] = pathIndex
    old_t = time.time()

    dx = 0.0
    dy = 0.0


    a=0
    delT=0
    while isRun:
        t1=time.time()
        if speed>road_speed:
            print("WARNING HIGH SPEED: ",index,speed)
        #if RL:
        #    print(absPos[index])
        temp_t = time.time()
        delT=temp_t-old_t
        old_t = time.time()

        posxOld=posx[index]
        posyOld=posy[index]
        x = x + dx*speed*(delT)
        y = y + dy*speed*(delT)
        posx[index] = x
        posy[index] = y
        velx[index] = dx*speed
        vely[index] = dy*speed
        talker(index,x,y,dx,dy,speed)
        inExp=False

        t2=time.time()
        if status == 192 or status == 193 or status == 200 or status == 197: #if in roundabout
           roundLength[index][0] = measureCircle(posx[index],posy[index],xRefR,yRefR,xCenterR,yCenterR,False)
           roundLength[index][1] = math.sqrt(velx[index]*velx[index]+vely[index]*vely[index])
           absPos[index]=roundLength[index][0]
           inExp=True
        else: #Not in roundabout
           roundLength[index][0] = 100 #some way of indicating that it should not be counted
           roundLength[index][1] = 100
        if status == 198: #if in queue (lower queue) (arc part)
           queueA[index][0] = measureCircle(posx[index],posy[index],QxRefA,QyRefA,QxCenterA,QyCenterA,True)
           queueA[index][1] = math.sqrt(velx[index]*velx[index]+vely[index]*vely[index])

           absPos[index] = (roundaboutLength + straightALength) + measureCircle(posx[index],posy[index],QxRefA2,QyRefA2,QxCenterA,QyCenterA,False)
           inExp=True
               
           if noInQueueA:
              inQueueA=inQueueA+1
              noInQueueA = False
        elif status ==90 and posx[index]<xStartA: #and xcondition: #straightaway of lower queue
           queueA[index][0]=queueArcLengthA+posx[index]-xvalueA
           queueA[index][1] = math.sqrt(velx[index]*velx[index]+vely[index]*vely[index])
           absPos[index]=roundaboutLength+abs(posx[index]-xStartA)
           if not enteredExp:
               randNum=random.random()

               if localP>=randNum:
               #if prob>=randNum and len(eAD)<1 and index==ind7:
               #if time.time()-timerA>50:%This condition create a RL vehicles after 50 S
                  #timerA=time.time()
                  RL=True
                  eAD.append(index)
                  enteredExp = True
           inExp=True
	   enteredExp=True
           if noInQueueA:
              inQueueA=inQueueA+1
              noInQueueA = False
        else: #Not in Queue
           queueA[index][0] = -1 #some way of indicating that it should not be counted
           queueA[index][1] = -1
           if not noInQueueA:
              inQueueA=inQueueA-1
              noInQueueA = True

        if status == 194: #if in queue arc of upper queue
           queueB[index][0] = measureCircle(posx[index],posy[index],QxRefB,QyRefB,QxCenterB,QyCenterB,False)
           queueB[index][1] = math.sqrt(velx[index]*velx[index]+vely[index]*vely[index])
           absPos[index] = (roundaboutLength + straightALength +arcALength+arcEBLength+straightEBLength+straightBLength) + measureCircle(posx[index],posy[index],QxRefB2,QyRefB2,QxCenterB,QyCenterB,True)
           inExp=True
           if noInQueueB:
              inQueueB=inQueueB+1
              noInQueueB = False
        elif (status == 74 or status == 73 or status == 41 or status == 36 or status == 30) and posy[index]>yStartB: #and ycondition: #straightaway of queue
           queueB[index][0]=queueArcLengthB+posy[index]-yvalueB
           queueB[index][1] = math.sqrt(velx[index]*velx[index]+vely[index]*vely[index])
           absPos[index]=(roundaboutLength + straightALength +arcALength+arcEBLength+straightEBLength)+abs(posy[index]-yStartB)

           if not enteredExp:
               if localP>=random.random():
               #if prob>=random.random() and len(eBD)<1 and index==ind3:#For deterministic RL
               #if time.time()-timerB>50:%This condition create a RL vehicles after 50 S
                  #timerB=time.time()
                  RL=True
                  eBD.append(index)
                  enteredExp = True
                  expStarted = True
           inExp=True
	   enteredExp=True
           if noInQueueB:
              inQueueB=inQueueB+1
              noInQueueB = False
        else: #Not in queue
           queueB[index][0] = -1 #some way of indicating that it should not be counted
           queueB[index][1] = -1
           if not noInQueueB:
              inQueueB=inQueueB-1
              noInQueueB = True

        if status == 195: #arc exiting upper (A103)
           absPos[index] = (roundaboutLength + straightALength +arcALength) + measureCircle(posx[index],posy[index],ExRefB,EyRefB,ExCenterB,EyCenterB,True)
           inExp=True
        elif (status == 75 or status == 72 or status == 40 or status ==34 or status==31) and posy[index]>(yExitB-straightEBLength) : #Straight exiting upper
           absPos[index] = (roundaboutLength + straightALength +arcALength+arcEBLength)+abs(posy[index]-yExitB)
           inExp=True

        elif (status == 74 or status == 73 or status == 41 or status == 36 or status == 30) and posy[index]<yStartB and posy[index]>yStartBUC: #and ycondition: #straightaway of queue
           queueB[index][0]=queueArcLengthB+posy[index]-yvalueB
           queueB[index][1] = math.sqrt(velx[index]*velx[index]+vely[index]*vely[index])
           absPos[index]=(roundaboutLength + straightALength +arcALength+arcEBLength+straightEBLength)+abs(posy[index]-yStartB)
        elif status ==90 and posx[index]>xStartA and posx[index]<xStartAUC: #and xcondition: #straightaway of lower queue
           queueA[index][0]=queueArcLengthA+posx[index]-xvalueA
           queueA[index][1] = math.sqrt(velx[index]*velx[index]+vely[index]*vely[index])
           absPos[index]=roundaboutLength+abs(posx[index]-xStartA)

        if status == 191: #arc exiting lower (A99)
           absPos[index] = (roundaboutLength + straightALength +arcALength+arcEBLength+straightEBLength+straightBLength+arcBLength) + measureCircle(posx[index],posy[index],ExRefA,EyRefA,ExCenterA,EyCenterA,True)
           inExp=True
        elif status == 91: #or status ==  or status == : #Straight exiting lower
           absPos[index] = (roundaboutLength + straightALength +arcALength+arcEBLength+straightEBLength+straightBLength+arcBLength+arcEALength)+abs(posx[index]-xExitA)
           inExp=True

        if not inExp:
           absPos[index]=-1
           enteredExp = False
           RL = False
           try:
               if eAD[0]==index:
                   eAD.popleft()
           except:
               if DEBUG:
                   print(str(index) + " is not in A deque")
           if eAD == None:
               eAD = deque()
           try:
               if eBD[0]==index:
                   eBD.popleft()
           except:
               if DEBUG:
                   print(str(index) + " is not in B deque")
           if eBD == None:
               eBD = deque()
        t3=time.time()


        rospy.wait_for_service('lf_grad')
        #Do not try to comment rospy.wait  to make the code faster, since it will break it ! ;)
        tt3 = time.time()
        try:
            resp = nf(status, x, y)
            tt4 = time.time()
            res = [resp.res, resp.dx, resp.dy]
            status = path.GetSegmentID(x,y)

            if res[0] == 0:
	        dx = res[1]
	        dy = res[2]
            elif res[0]==2:
                pass
            else:
	        print "Zumo "+str(index)+" Cannot Run NF."
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        if True:
            t4=time.time()
            #if index == ind1:
                #print 'wait ' + str(tt3-t3) + ' service ' + str(tt4-tt3) + ' ifelse ' + str(t4-tt4)
            if inExp:#status == 192 or status == 193 or status == 197 or status == 200 or status == 194 or status == 195 or status == 74 or status == 41 or status == 73 or status == 90 or status == 198: #Experiement zone
               if RL:
                   if eAD and index == eAD[0]:
                       a = NN_a1[0]
                       speed = min(max(0,speed + a*delT),road_speed) #Saturates velocity and permits it to not go below 0. Not ideal, but necessary functionality currently. May change in future.
                   elif eBD and index == eBD[0]:
                       a = NN_a2[0]
                       speed = min(max(0,speed + a*delT),road_speed)
                   else:
                       speed = getIDMSpeed(index,speed,delT,road_speed,pathIndex)
               else: #Use car following model
                 try:
                     speed=getIDMSpeed(index,speed,delT,road_speed,pathIndex)
                 except:
                     #isRun = False
                     print("Fatal Error with IDM, headway probably read 0")


            else:
               speed=getIDMSpeed(index,speed,delT,road_speed,pathIndex)
            t5=time.time()
        #Yield Signs
        t6=time.time()
        if posx[index]<xmaxA and posx[index]>xminA and posy[index]>yminA and posy[index]<ymaxA and status == 198: #If you are at the yield sign
           if RL:
               if index !=eAD[0]:
                   speed=trafficYield(x_minA,x_maxA,y_minA,y_maxA,speed,posx,posy)
           else: #You are not an RL vehicle. Yield.
               speed=trafficYield(x_minA,x_maxA,y_minA,y_maxA,speed,posx,posy)
        if posx[index]<xmaxB and posx[index]>xminB and posy[index]>yminB and posy[index]<ymaxB: #If you are at the yield sign
           if RL:
               if index !=eBD[0]: #if you are not an active RL vehicle
                   speed=trafficYield(x_minB,x_maxB,y_minB,y_maxB,speed,posx,posy)
           else:
               speed=trafficYield(x_minB,x_maxB,y_minB,y_maxB,speed,posx,posy)
        ######## STOP SIGNS ########
        elif index == indiciesA[0] and inZone(stopA,stopA+stopDistance,stopy_minA,stopy_maxA,posx[index],posy[index]): #Stop signs
        
            if stopOn:
                speed = 0.0
                leadInStopA=True
                flush_a=False
                if not stopped: #First time at stop sign
                    n_A=random.randrange(a_min,a_max,1) 
                    stopped = True
                    print("n_A: " + str(n_A))
            else:
                stopped = False
            
        elif index==indiciesA[n_A] and inZone(stopA,stopA+stopDistance,stopy_minA,stopy_maxA,posx[index],posy[index]) and not flush_a:
            speed=0.0
        elif index == indiciesB[0] and inZone(stopx_minB,stopx_maxB,stopB-stopDistance,stopB,posx[index],posy[index]): #Stop signs
            if stopOn:
                speed = 0.0
                leadInStopB=True
                flush_b=False
                if not stopped: #First time at stop sign
                    n_B=random.randrange(b_min,b_max,1)
                    stopped = True   
                    print("n_B: " + str(n_B))
            else:
                stopped = False
        elif index==indiciesB[n_B] and inZone(stopx_minB,stopx_maxB,stopB-stopDistance,stopB,posx[index],posy[index]) and not flush_b:
            speed=0.0
        t7=time.time()

        if stopOn and experimentDone:
            if leadInStopA and leadInStopB:
                if index==indiciesA[0]:
                    if abs(posx[indiciesA[0]]-posx[indiciesA[n_A-1]]) <=n_A*carDistance:
                        platoonAReady = True
                    else:
                        platoonAReady = False
                if index==indiciesB[0]:
                    if abs(posy[indiciesB[0]]-posy[indiciesB[n_B-1]]) <=n_B*carDistance:
                        platoonBReady = True
                    else:
                        platoonBReady = False
            if platoonAReady and platoonBReady and max(absPos) == -1:
                stopOn = False
                experimentDone = False
                leadInStopB = False
                leadInStopA = False
                platoonAReady = False
                platoonBReady = False

        if not experimentDone:
            if expStarted and max(absPos) == -1:
                experimentDone = True
                flush_a=True
                flush_b=True
                stopB=random.uniform(stopBmin,stopBmax)
                stopA=random.uniform(stopAmin,stopAmax)
                print("Flush Started")
                stopOn =True
                expStarted=False
        #if index==ind1:
            #print "t2-t1 = "+str(t2-t1)+"t3-t2 = "+str(t3-t2)+"t4-t3 = "+str(t4-t3)+"t5-t4 = "+str(t5-t4)+"t6-t5 = "+str(t6-t5)+"t7-t6 = "+str(t7-t6)
        sleepytime = time.time() - temp_t
        time_c=time_c+sleepytime
        num=num+1
        if sleepytime < controlRate:
	        sleepytime = controlRate - sleepytime
        else:
            #print(sleepytime)
            sleepytime = 0
            #print("Warning: Extended past standard control rate time")
        time.sleep(sleepytime)
        
rospy.init_node('zumo_go2')#, anonymous=True)

pub = rospy.Publisher('/ZumoRefs', String, queue_size=1000)
#headwayManager = Headway2.Headway([[-1.952,-.9],[-1.952,-2],[1.1,1.22]],[29,32,31],'lf_grad',False)
headwayManager = Headway2.Headway([[-1.952,-.9],[2.238,1.22]],paths,'lf_grad',False)
#[[-1.952,-.9],[-1.952,-2],[2.238,1.22],[2.238,1.22],[-1.952,-2]]
path1X,path1Y = pathplotbuilder(headwayManager.nodes[29])
#path2X,path2Y = pathplotbuilder(headwayManager.nodes[32])
#path3X,path3Y = pathplotbuilder(headwayManager.nodes[31])
path4X,path4Y = pathplotbuilder(headwayManager.nodes[30])
##Path 13

###RL
global control
control=RLController("/home/themainframe/catkin_ws/src/line_following/script/flow-transfer-master/data/weights/varied_inflows_both_noise_2.pkl")
#roundabout_89_2018_10_17_15_25_04_0003.pkl LAST POLICY WITHOUT NOISE
#roundabout_79_2018_10_15_16_35_06_0003.pkl WITH NOISE

###NEWEST POLICIES:####
# varied_inflows_no_noise_0.pkl


pathIndexTable = [0]*36




tHW = Thread(target = headwayMaster, args = [paths])
tHW.start()

#tVD = Thread(target = viconDataMaster)
#tVD.start()

time.sleep(1)

#The last number in arg correspons to Pathindex
t1 = Thread(target = zumoThread, args = (ind1, "zumoTest"+str(ind1), rate, copy.deepcopy(GetDefaultPath(41)),29))#For Path 34,35,36,37 the nodes are being created same as path 32
t1.start()
t2 = Thread(target = zumoThread, args = (ind2, "zumoTest"+str(ind2), rate, copy.deepcopy(GetDefaultPath(34)),29))
t2.start()

t3 = Thread(target = zumoThread, args = (ind3, "zumoTest"+str(ind3), rate, copy.deepcopy(GetDefaultPath(34)),29))
t3.start()
t4 = Thread(target = zumoThread, args = (ind4, "zumoTest"+str(ind4), rate, copy.deepcopy(GetDefaultPath(34)),29))
t4.start()
t5 = Thread(target = zumoThread, args = (ind5, "zumoTest"+str(ind5), rate, copy.deepcopy(GetDefaultPath(34)),29))
t5.start()

t6 = Thread(target = zumoThread, args = (ind6, "zumoTest"+str(ind6), rate, copy.deepcopy(GetDefaultPath(43)),30))
t6.start()

t7 = Thread(target = zumoThread, args = (ind7, "zumoTest"+str(ind7), rate, copy.deepcopy(GetDefaultPath(42)),30))
t7.start()
t8 = Thread(target = zumoThread, args = (ind8, "zumoTest"+str(ind8), rate, copy.deepcopy(GetDefaultPath(42)),30))
t8.start()

t9 = Thread(target = zumoThread, args = (ind9, "zumoTest"+str(ind9), rate, copy.deepcopy(GetDefaultPath(39)),30))
t9.start()
t10 = Thread(target = zumoThread, args = (ind10, "zumoTest"+str(ind10), rate, copy.deepcopy(GetDefaultPath(39)),30))
t10.start()
t11 = Thread(target = zumoThread, args = (ind11, "zumoTest"+str(ind11), rate, copy.deepcopy(GetDefaultPath(39)),30))
t11.start()
t12 = Thread(target = zumoThread, args = (ind12, "zumoTest"+str(ind12), rate, copy.deepcopy(GetDefaultPath(39)),30))
t12.start()
t13 = Thread(target = zumoThread, args = (ind13, "zumoTest"+str(ind13), rate, copy.deepcopy(GetDefaultPath(39)),30))
t13.start()
#For Path 38,39 the nodes are being created same as path 30

if (SAVE_DATA):
    twrite = Thread(target = csvout)
    twrite.start()


tNN = Thread(target = NNMaster)
tNN.start()

#ioff()
ax1.plot(path1X,path1Y,'b')
#ax1.plot(path2X,path2Y,'g')
#ax1.plot(path3X,path3Y,'r')
ax1.plot(path4X,path4Y,'y')







ani = animation.FuncAnimation(fig, animate, interval=100)
ax1.set_xlim([3.048,-3.548])
ax1.set_ylim([1.524,-5])
plt.axis('equal')
plt.show()

isRun = False
timeAvg=time_c/num
print(timeAvg)
