#Headway Return
import rospy
import tf
import math
import time
import datetime
import numpy as np
from numpy.linalg import inv,pinv
from geometry_msgs.msg import TransformStamped
import matplotlib.pyplot as plt
from std_msgs.msg import String
from socketIO_client import SocketIO, LoggingNamespace

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

def getHeadway(follower,leader): 
	#Returns the Headway between the lead vehicle and the following vehicle
	followerX,followerY,followerT=getViconPos(follower)
	leaderX,leaderY,leaderT=getViconPos(leader)
	
