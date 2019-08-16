#!/usr/bin/env python

import rospy
import tf
import math
import time
import numpy as np
from numpy.linalg import inv,pinv
from geometry_msgs.msg import TransformStamped
import matplotlib.pyplot as plt


def pathplotbuilder(Node):
#Node is in form [[x,y],[x,y]...]
#Turns a list of x,y coordinates into seperate lists of x and y values
	x=[0]*len(Node)
	y=[0]*len(Node)
	for i in range(len(Node)):
		x[i-1]=Node[i-1][0]
		y[i-1]=Node[i-1][1]
	return x,y



