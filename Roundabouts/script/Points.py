#!/usr/bin/env python

import rospy
import tf
import math
import time
import numpy as np
from numpy.linalg import inv,pinv
from geometry_msgs.msg import TransformStamped
import matplotlib.pyplot as plt

######################################################################################################
#                                                FUNCTIONS                                           
######################################################################################################

#################################################
#                STRAIGHT PATH   
#################################################
def straightPath(vd,dt,xdo,ydo,xdf,ydf):
    global switch
    global xRef,yRef,j
    global looptime
    thetad = math.atan2(ydf-ydo,xdf-xdo)
    
    # Parametric equations for a straight line
    xd = xdo + vd*looptime*math.cos(thetad)
    xd_d = vd*looptime*math.cos(thetad)
    xd_dd = 0
    yd = ydo + vd*looptime*math.sin(thetad)
    yd_d = vd*looptime*math.sin(thetad)
    yd_dd = 0

    if np.sqrt(np.power(xd-xdo,2)+np.power(yd-ydo,2))>np.sqrt(np.power(xdf-xdo,2)+np.power(ydf-ydo,2)):
	switch = 1

    xRef.insert(j,xd)
    yRef.insert(j,yd)
    j = j +1
    looptime = looptime + dt

#################################################
#                    ARC PATH 
#################################################
def arcPath(vd,dt,ccw,xc,yc,xdo,ydo,xdf,ydf):
    global switch
    global xRef,yRef,j
    global looptime
    global pi
    
    # First rotate local frame to align with initial position (never crossed 360 this way)
    theta_lFrame = math.atan2(ydo-yc,xdo-xc)  
    if theta_lFrame<0:
	theta_lFrame = theta_lFrame + 2*pi
    #theta_lFrame = ccw*theta_lFrame

    r = np.sqrt(np.power(xc-xdo,2)+np.power(yc-ydo,2))

    thetac=vd*looptime/r #First how much the robot has moved wrt the local center frame and xo axis
    x_lFrame = r*math.cos(ccw*thetac) #Effectively this is in the local rotated coord system by thetac
    y_lFrame = r*math.sin(ccw*thetac) #ccw is one (true) or negative one (false - clockwise)
    
    #Rotation matrix and its derivative to get back to global frame 
    xd=xc + x_lFrame*math.cos(theta_lFrame) + y_lFrame*-math.sin(theta_lFrame)
    xd_d=x_lFrame*-math.sin(theta_lFrame) + y_lFrame*-math.cos(theta_lFrame)
    xd_dd=x_lFrame*-math.cos(theta_lFrame) + y_lFrame*math.sin(theta_lFrame)
    yd=yc + x_lFrame*math.sin(theta_lFrame) + y_lFrame*math.cos(theta_lFrame)
    yd_d=x_lFrame*math.cos(theta_lFrame) + y_lFrame*-math.sin(theta_lFrame)
    yd_dd=x_lFrame*-math.sin(theta_lFrame) + y_lFrame*-math.cos(theta_lFrame)

    #Determine arc length from starting point to ending point (this can be computed once beforehand)
    xdf_lFrame=(xdf-xc)*math.cos(theta_lFrame) + (ydf-yc)*math.sin(theta_lFrame)
    ydf_lFrame=(xdf-xc)*-math.sin(theta_lFrame) + (ydf-yc)*math.cos(theta_lFrame)
    thetaf_lFrame = math.atan2(ydf_lFrame,xdf_lFrame)
    if thetaf_lFrame<0:
	thetaf_lFrame = thetaf_lFrame + 2*pi

    # Global desired angle, velocity, angluar velocity of the robot frame
    thetad = math.atan2(yd_d,xd_d)
    if thetad<0:
	thetad = thetad + 2*pi
    #print("xd: " + str(xd) + " yd: " +str(yd))
    vd = np.sqrt(np.power(xd_d,2)+np.power(yd_d,2))
    wd = (yd_dd*np.transpose(xd_d)-xd_dd*np.transpose(yd_d))/(np.power(xd_d,2)+np.power(yd_d,2))

    #print("thetac: " + str(thetac) +" thetaf_lFrame: " + str(thetaf_lFrame))
    if ccw == 1 and thetac>thetaf_lFrame: #causes overshoot, so anticipate and end early with -v*dt?
	switch = 1
    elif ccw == -1 and thetac>2*pi-thetaf_lFrame: #causes overshoot, so anticipate and end early with -v*dt?
	switch = 1

    xRef.insert(j,xd)
    yRef.insert(j,yd)
    j = j +1
    looptime = looptime + dt
    
#################################################
#                  CW ARC PATH 
#################################################
def arcPathCW(vd,dt,ccw,xc,yc,xdo,ydo,xdf,ydf):
    global switch
    global xRef,yRef,j
    global looptime
    global pi
    
    # First rotate local frame to align with initial position (never crossed 360 this way)
    theta_o = math.atan2(ydo-yc,xdo-xc)  #the order matters!
    #print(" theta_o should be about zero: " + str(theta_o)) #YES
    if theta_o<=0:
	theta_o = theta_o + 2*pi
    #print(" theta_o should be about 2pi: " + str(theta_o)) #YES

    r = np.sqrt(np.power(xc-xdo,2)+np.power(yc-ydo,2))

    thetac=-vd*looptime/r #First how much the robot has moved wrt the local center frame and xo axis
    xd = xc + r*math.cos(theta_o+thetac)
    xd_d = r*math.sin(theta_o+thetac)
    xd_dd = -r*math.cos(theta_o+thetac)
    yd = yc + r*math.sin(theta_o+thetac)
    yd_d = -r*math.cos(theta_o+thetac)
    yd_dd = r*math.sin(theta_o+thetac)

    #Determine arc length from starting point to ending point (this can be computed once beforehand)
    thetaf_lFrame = math.atan2(ydf-yc,xdf-xc)
    #print(" thetaf_l should be -pi/2: " + str(thetaf_lFrame)) #YES
    if thetaf_lFrame<0:
	thetaf_lFrame = thetaf_lFrame + 2*pi
    #print(" thetaf_l should be 3pi/2: " + str(thetaf_lFrame)) #YES

    # Global desired angle, velocity, angluar velocity of the robot frame
    thetad = math.atan2(yd_d,xd_d)
    vd = np.sqrt(np.power(xd_d,2)+np.power(yd_d,2))
    wd = (yd_dd*np.transpose(xd_d)-xd_dd*np.transpose(yd_d))/(np.power(xd_d,2)+np.power(yd_d,2))

    #print("thetac: " + str(thetac) +" thetaf_lFrame: " + str(thetaf_lFrame))
    if abs(thetac)>(theta_o-thetaf_lFrame): #causes overshoot, so anticipate and end early with -v*dt?
	switch = 1

    xRef.insert(j,xd)
    yRef.insert(j,yd)
    j = j +1
    looptime = looptime + dt


def pathBuilder(vd,dt):
    print("Generating Reference Path")
    global j
    global xRef, yRef
    global switch
    global looptime
    global pi
    pi = 3.1415

    ######     PATH 0      ####################################################################################################
    xRef = []
    yRef = []
    j = 1
    xdo = 0.8128
    ydo = 0.3048
    xRef.insert(0,xdo)
    yRef.insert(0,ydo)

    ####################################################################################
    #################################     S94      #####################################
    ####################################################################################
    switch = 0
    looptime = 0
    print "path S94"
    while switch == 0:
        straightPath(vd,dt,0.8128,0.3048,3.9224,0.3064) #straightPath(vd,dt,xdo,ydo,xdf,ydf)
    
    ####################################################################################
    #################################     A106      ####################################
    ####################################################################################
    switch = 0
    looptime = 0
    print "path A106"
    while switch == 0:
        arcPath(vd,dt,1,3.8100,4.2164,3.9224,0.3064,5.3160,0.6063) #arcPath(vd,dt,xc,yc,xdo,ydo,xdf,ydf)
    
    ####################################################################################
    ###########################     A105/A101/A100      ################################
    ####################################################################################
    switch = 0
    looptime = 0
    print "path A105/101/100"
    while switch == 0:
        arcPath(vd,dt,1,5.0800,1.1684,5.3160,0.6063,4.4752,1.2483) #arcPath(x,y,theta,vd,to,xc,yc,xdo,ydo,xdf,ydf)

    ####################################################################################
    #################################     A99      #####################################
    ####################################################################################
    switch = 0
    looptime = 0
    print "path A99"
    while switch == 0:
        arcPathCW(vd,dt,-1,3.9224,1.3198,4.4752,1.2483,3.9224,0.7624) #arcPath(x,y,theta,vd,to,xc,yc,xdo,ydo,xdf,ydf)
    
    ####################################################################################
    #################################     S91      #####################################
    ####################################################################################
    switch = 0
    looptime = 0
    print "path S91"
    while switch == 0:
        straightPath(vd,dt,3.9224,0.7624,1.9279,0.7653) #straightPath(x,y,theta,vd,to,xdo,ydo,xdf,ydf)#3.9224
    
    ####################################################################################
    ##################################     A92      ####################################
    ####################################################################################
    switch = 0
    looptime = 0
    print "path A92"
    while switch == 0:
        arcPathCW(vd,dt,-1,1.8904,1.2954,1.9279,0.7653,1.3936,1.1069) #arcPath(x,y,theta,vd,to,xc,yc,xdo,ydo,xdf,ydf)
    
    ####################################################################################
    ##############################     A87/A84/A81      ################################
    ####################################################################################
    switch = 0
    looptime = 0
    print "path A87/84/81"
    while switch == 0:
        arcPath(vd,dt,1,0.8128,0.9144,1.3936,1.1069,0.8128,0.3048) #arcPath(x,y,theta,vd,to,xc,yc,xdo,ydo,xdf,ydf)

    #For visualization
    #plt.plot(xRef,yRef,'r--')
    #plt.gca().set_aspect('equal', adjustable='box')
    #plt.show()
    

    xOrigin = 3.048 # from map corner
    yOrigin = 1.524 # from map bottom corner
    for i in range(len(xRef)):
        xRef[i] = -1*(xRef[i]-xOrigin)
        yRef[i] = -1*(yRef[i]-yOrigin)

    xRef0 = xRef
    yRef0 = yRef

    ######     PATH 1      ####################################################################################################
    xRef = []
    yRef = []
    j = 1
    xdo = 1.0414
    ydo = 2.5146
    xRef.insert(0,xdo)
    yRef.insert(0,ydo)

    ####################################################################################
    #################################     S61      #####################################
    ####################################################################################
    switch = 0
    looptime = 0
    print "path S94"
    while switch == 0:
        straightPath(vd,dt,1.0414,2.5146,2.667,2.5146) #straightPath(vd,dt,xdo,ydo,xdf,ydf)
    
    ####################################################################################
    #################################     A58       ####################################
    ####################################################################################
    switch = 0
    looptime = 0
    print "path A106"
    while switch == 0:
        arcPathCW(vd,dt,-1,2.667,2.3622,2.667,2.5146,2.8194,2.3622) #arcPath(vd,dt,xc,yc,xdo,ydo,xdf,ydf)

    ####################################################################################
    #################################     S80      #####################################
    ####################################################################################
    switch = 0
    looptime = 0
    print "path S94"
    while switch == 0:
        straightPath(vd,dt,2.8194,2.3622,2.8194,1.2192) #straightPath(vd,dt,xdo,ydo,xdf,ydf)
    
    ####################################################################################
    #################################     A94       ####################################
    ####################################################################################
    switch = 0
    looptime = 0
    print "path A106"
    while switch == 0:
        arcPathCW(vd,dt,-1,2.3622,1.2192,2.8194,1.2192,2.41115835,0.76462886) #arcPath(vd,dt,xc,yc,xdo,ydo,xdf,ydf)

    ####################################################################################
    ##############################     S95/91/89      ##################################
    ####################################################################################
    switch = 0
    looptime = 0
    print "path S94"
    while switch == 0:
        straightPath(vd,dt,2.41115835,0.76462886,1.92790175,0.76533227) #straightPath(vd,dt,xdo,ydo,xdf,ydf)
    
    ####################################################################################
    #################################     A92       ####################################
    ####################################################################################
    switch = 0
    looptime = 0
    print "path A106"
    while switch == 0:
        arcPathCW(vd,dt,-1,1.89043073,1.2954,1.92790175,0.76533227,1.39359333,1.10691002) #arcPath(vd,dt,xc,yc,xdo,ydo,xdf,ydf)

    ####################################################################################
    ##############################     A87/84/81       #################################
    ####################################################################################
    switch = 0
    looptime = 0
    print "path A106"
    while switch == 0:
        arcPath(vd,dt,1,0.8128,0.9144,1.39359333,1.10691002,1.05111487,1.47548661) #arcPath(vd,dt,xc,yc,xdo,ydo,xdf,ydf)
    
    ####################################################################################
    #################################     A86       ####################################
    ####################################################################################
    switch = 0
    looptime = 0
    print "path A106"
    while switch == 0:
        arcPathCW(vd,dt,-1,1.16537619,1.7272,1.05111487,1.47548661,0.889,1.7272) #arcPath(vd,dt,xc,yc,xdo,ydo,xdf,ydf)

    ####################################################################################
    #################################     S88      #####################################
    ####################################################################################
    switch = 0
    looptime = 0
    print "path S94"
    while switch == 0:
        straightPath(vd,dt,0.889,1.7272,0.889,2.3622) #straightPath(vd,dt,xdo,ydo,xdf,ydf)
    
    ####################################################################################
    #################################     A53       ####################################
    ####################################################################################
    switch = 0
    looptime = 0
    print "path A106"
    while switch == 0:
        arcPathCW(vd,dt,-1,1.0414,2.3622,0.889,2.3622,1.0414,2.5146) #arcPath(vd,dt,xc,yc,xdo,ydo,xdf,ydf)

    for i in range(len(xRef)):
        xRef[i] = -1*(xRef[i]-xOrigin)
        yRef[i] = -1*(yRef[i]-yOrigin)

    xRef1 = xRef
    yRef1 = yRef

    ######     PATH 0 Control Region      #######################################################################################
    xRef = []
    yRef = []
    j = 1
    xdo = 4.0003039
    ydo = 0.7735917
    xRef.insert(0,xdo)
    yRef.insert(0,ydo)

    ####################################################################################
    #################################     A53c       ###################################
    ####################################################################################
    switch = 0
    looptime = 0
    print "path A53c"
    while switch == 0:
        arcPathCW(vd,dt,-1,3.9224303,1.31983238,4.0003039,0.7735917,3.92243073,0.76242912) #arcPath(vd,dt,xc,yc,xdo,ydo,xdf,ydf)

    ####################################################################################
    ##############################     S95/91/89c      #################################
    ####################################################################################
    switch = 0
    looptime = 0
    print "path S95/91/89c"
    while switch == 0:
        straightPath(vd,dt,3.92243073,0.76242912,2.41115835,0.76242912) #straightPath(vd,dt,xdo,ydo,xdf,ydf)

    for i in range(len(xRef)):
        xRef[i] = -1*(xRef[i]-xOrigin)
        yRef[i] = -1*(yRef[i]-yOrigin)

    xRef0c = xRef
    yRef0c = yRef

    ######     PATH 1 Control Region      #######################################################################################
    xRef = []
    yRef = []
    j = 1
    xdo = 2.8194
    ydo = 2.2192
    xRef.insert(0,xdo)
    yRef.insert(0,ydo)

    ####################################################################################
    #################################     S80c      ####################################
    ####################################################################################
    switch = 0
    looptime = 0
    print "path S80c"
    while switch == 0:
        straightPath(vd,dt,2.8194,2.2192,2.8194,1.2192) #straightPath(vd,dt,xdo,ydo,xdf,ydf)
    
    ####################################################################################
    #################################     A94c       ###################################
    ####################################################################################
    switch = 0
    looptime = 0
    print "path A94c"
    while switch == 0:
        arcPathCW(vd,dt,-1,2.3622,1.2192,2.8194,1.2192,2.41115835,0.76462886) #arcPath(vd,dt,xc,yc,xdo,ydo,xdf,ydf)


    for i in range(len(xRef)):
        xRef[i] = -1*(xRef[i]-xOrigin)
        yRef[i] = -1*(yRef[i]-yOrigin)

    xRef1c = xRef
    yRef1c = yRef

    ######     Merge Region      #######################################################################################
    xRef = []
    yRef = []
    j = 1
    xdo = 2.41115835
    ydo = 0.76242912
    xRef.insert(0,xdo)
    yRef.insert(0,ydo)

    ####################################################################################
    #############################     S95/91/89m      ##################################
    ####################################################################################
    switch = 0
    looptime = 0
    print "path S80c"
    while switch == 0:
        straightPath(vd,dt,2.41115835,0.76242912,2.41115835-0.3,0.76242912) #straightPath(vd,dt,xdo,ydo,xdf,ydf)

    for i in range(len(xRef)):
        xRef[i] = -1*(xRef[i]-xOrigin)
        yRef[i] = -1*(yRef[i]-yOrigin)

    xRefm = xRef
    yRefm = yRef


    return xRef0,yRef0,xRef1,yRef1,xRef0c,yRef0c,xRef1c,yRef1c,xRefm,yRefm

#vd = 0.2
#dt = 0.01

#xRef,yRef = pathBuilder(vd,dt)

#print "done!"
#print xRef
#print yRef

#For visualization
#plt.plot(xRef,yRef,'r--')
#plt.gca().set_aspect('equal', adjustable='box')
#plt.show()

def BuildPath(vl):
    vd = 1
    dt = vl
    return pathBuilder(vd,dt)

