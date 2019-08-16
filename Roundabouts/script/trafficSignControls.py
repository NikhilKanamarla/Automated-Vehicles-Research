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

def trafficYield(x_min,x_max,y_min,y_max,road_speed,posx,posy):
#Function simulates yielding to traffic in a given zone.
	doYield = False
	for i in range(1,len(posx)): #For every vehicle
		if posx[i]>x_min and posx[i]<x_max and posy[i]>y_min and posy[i]<y_max: #Check if it is in area you are yielding to
			doYield = True; #There is at least one car in the yield to zone

	if doYield:
		#print("Yield")
		return 0.0
	else:
		return road_speed

def inZone(x_min,x_max,y_min,y_max,posx,posy):
	if posx>x_min and posx<x_max and posy>y_min and posy<y_max:
		return True
	else:
		return False

