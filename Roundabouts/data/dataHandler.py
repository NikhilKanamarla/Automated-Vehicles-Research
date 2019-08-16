#!/usr/bin/env python

import rospy
import tf
import math
import time
import datetime ##
import copy
import numpy as np
import csv
from numpy.linalg import inv,pinv
from threading import Thread, Lock
from geometry_msgs.msg import TransformStamped
from line_following.srv import *
from socketIO_client import SocketIO, LoggingNamespace
from io import StringIO ##
import matplotlib.pyplot as plt

###################    Find closest point    ###########################
#path = 1 
#posX = .5
#posY = .1
def closestPoint(path,posX,posY):
   radius2 = 18*0.0254 #convert to meters from inches
   radius1 = 21.945*0.0254
   radX1 = -24.43 * .0254
   radY1 = 8.04 * .0254
   radX2 = 27 * .0254
   radY2 = 12 * .0254

   global totalDistance
   global dline
   #Chose path(code assumes vehicle is already in control zone)
   if path == 0:
      #indicates whether on straight or arc. 
      if posX<-34.426*.0254:
	 #Formula to find closest point on the arc
         Cx = radX1 + radius1 * abs((posX-radX1)/numpy.sqrt(((posX-radX1)**2)+((posY-radY1)**2)))
	 Cy = radY1 + radius1 * abs((posY-radY1)/numpy.sqrt(((posX-radX1)**2)+((posY-radY1)**2)))
	 #Solves for theta using the arc point and radius point
         thetaArc = math.atan((Cy-radY1)/(Cx-radX1))
	 #Solves for arc length based on theta and radius
	 dArc = thetaArc*radius1
      if posX>=-34.426*.0254:
	 #Solves for distance from start of line based on x values
	 dLine = math.abs(posX-originLineX)
      totalDistance = dLine + dArc
   if path == 1:
      if posY>-12*.0254:
	 Cx = radX2 + radius2 * abs((posX-radX2)/numpy.sqrt(((posX-radX2)**2)+((posY-radY2)**2)))
	 Cy = radY2 + radius2 * abs((posY-radY2)/numpy.sqrt(((posX-radX2)**2)+((posY-radY2)**2)))
	 thetaArc = math.atan((Cy-radY2)/(Cx-radX2))
	 dArc = thetaArc*radius2
	 dLine = 0
      if posY<=-12*.0254:
	 dline = math.abs(posY-originLineY)
      totalDistance = dLine + dArc

   return totalDistance #distance traveled

#######################   Post processing   #########################
#Right click on the file in whatever folder you stored it in and hit copy then paste to paste file name between "" below
#filename = "/home/themainframe/catkin_ws/src/line_following/data/data_10_20_17__13_25_39.csv"
filename = "/home/themainframe/catkin_ws/src/line_following/data/goodDataExamples/data_10_15_17__22_23_29.csv"
#/home/themainframe/catkin_ws/src/line_following/data/data_10_19_17__18_41_56.csv
reader = csv.reader(open(filename))
datMat = list(reader)
entries = len(datMat)
print datMat
print entries

carIDs = []
for i in range(entries-3):
    if datMat[i][1] not in carIDs:
        carIDs.append(datMat[i][1])
carIDs.sort() #order smallest to largest

carDataMat=[]
for j in range(len(carIDs)):
    tempMat = []
    for i in range(entries-20):
        if datMat[i][1] == carIDs[j]:
            tempMat.append(datMat[i])
    carDataMat.append(tempMat)

for j in range(len(carIDs)): #using cur_vel
    print carIDs
    time=[]
    dtime=[]
    vel=[]
    velc=[]
    bat11=[]
    bat6=[]
    xPos = []
    yPos = []
    pre_Vel = 0.30
    xPre = 0
    yPre = 0   
    tPre = 0
    for k in range(len(carDataMat[j])):
        #if float(carDataMat[j][k][4])>0.01 and float(carDataMat[j][k][4])<10 and abs(float(carDataMat[j][k][4])-pre_Vel)<0.03:
        if float(carDataMat[j][k][4])<10:
            xNow = float(carDataMat[j][k][2])
            yNow = float(carDataMat[j][k][3])
            tNow = float(carDataMat[j][k][0])
            dtime.append(tNow-tPre)
            velNow = ((xNow-xPre)*(xNow-xPre)+(yNow-yPre)*(yNow-yPre))**0.5 / (tNow - tPre)
            #velNow = (yNow-yPre)/(tNow-tPre)
            #vel.append(velNow)
            #vel.append(float(carDataMat[j][k][4]))
            xPos.append(xNow)
            yPos.append(yNow)
            pre_Vel = float(carDataMat[j][k][4])
            if j == 0:
                print (tNow - tPre)
            xPre = xNow
            yPre = yNow
            tPre = tNow
        #Vel plots
        '''if j == 16:#plot individual paths
            time.append(tNow)
            vel.append(velNow)
            velc.append(float(carDataMat[j][k][4])*0.87336)
            plt.scatter(time,vel,color="blue")
            plt.scatter(time,velc,color="green")
            #plt.scatter(xPos,yPos)
        if j == 6: #plot individual paths
            time.append(tNow)
            vel.append(float(carDataMat[j][k][4]))
            plt.plot(time,vel,color="red")
            #plt.scatter(xPos,yPos, color = 'red') 
	    #plt.scatter(time,vel,color="purple")'''

        #Battlvl plots
        if j == 11: #plot individual paths
            time.append(tNow)
            bat11.append(float(carDataMat[j][k][5]))
            plt.plot(time,bat11,color="red")
            #plt.scatter(xPos,yPos, color = 'red') 
	    #plt.scatter(time,vel,color="purple")
        if j == 16: #plot individual paths
            time.append(tNow)
            bat6.append(float(carDataMat[j][k][5]))
            plt.plot(time,bat6,color="red")
            #plt.scatter(xPos,yPos, color = 'red') 
	    #plt.scatter(time,vel,color="purple")
print entries

plt.show()
