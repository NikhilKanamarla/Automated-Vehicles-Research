# Meant to be created once per sim, at the top of code.

import math
from Points import BuildPath
import numpy as np
from numpy.linalg import inv,pinv
from threading import Thread, Lock
from line_following.srv import *
import rospy
from Path import GetDefaultPath
import copy

class Headway(object):
	''' An object that handles all of the headway and tailway work '''

	def __init__(self, init_pos, paths, map_builder, debug = False):
		''' Initializes the Headway handler by generating the nodes '''
		# init_pos is an initial point on a given corresponding paths
		# eg: init_pos[0] -> any spot on paths[0]
		#init_pos[[5,10],[2,17],[7,10]]
		#paths = [41, 46, 59]

		self.map_builder = map_builder; # Map builder (eg: 'lf_grad')
		self.s = rospy.ServiceProxy(map_builder, LineFollowing) # Define srvice
		self.speed = 0.01 # Misnomer, needs fix. It's distance traveled with each tick
		self.DEBUG = debug # Enables verbose console messages
		if(self.DEBUG):
			rospy.loginfo("Debug Mode is ON")
			print("Debug mode is on")

		# Generate each series of nodes using initial position and path
		# usage: In order to call nodes for a given path, do self.nodes[pathid]
		self.nodes = dict()

		#TODO: Add check for init_pos=paths

		# Node generation for each path using the initial position
		for i in range(len(paths)):
			self.nodes[paths[i]] = self.generateNodes(init_pos[i], paths[i])#Behdad: i-1 to i since it starts from 0


		if(self.DEBUG):
			rospy.loginfo(str(self.nodes))



	def generateNodes(self, pos, path_index):
		''' Generates the track of nodes for a given path '''
		path=copy.deepcopy(GetDefaultPath(path_index))
		nodes = [] # List of nodes
		end = False #TODO: Checks if entire path is logged, idea: loop twice? segment_id?

		tickCount = 1500 # Temp variable in replace of end
		if(self.DEBUG):
			rospy.loginfo("Started node generation")
		nodes.append(pos)
		status = path.GetSegmentIDNoCheck()
		indexOld=path.index
		continueLoop = True
		i = 0
		#for i in range(tickCount): #TODO Replace with end check
		while continueLoop:
			rospy.wait_for_service(self.map_builder)
			i=i+1
			#if(False):#selfDEBUG):
			rospy.loginfo("i value is " + str(i))

			resp = self.s(status, pos[0], pos[1])
			vec = [resp.res, resp.dx, resp.dy]
			if(vec[0]==0):
				dx = vec[1]
				dy = vec[2]
			elif vec[0]==2:
				pass

			status = path.GetSegmentID(pos[0],pos[1])
			# Move the ping and append to list
			pos = [pos[0] + self.speed*dx, pos[1] + self.speed*dy]
			nodes.append(pos)
			indexNew=path.index
			if(self.DEBUG):
				rospy.loginfo("IndexNew" + str(indexNew))
			dif=indexNew-indexOld
			if(self.DEBUG):
				rospy.loginfo("Difference is " + str(dif))
			indexOld=indexNew
			if dif<0:
				continueLoop = False

		if(self.DEBUG):
			rospy.loginfo("Nodes for path " + str(path) + ": " + str(nodes))

		return nodes

	def findHeadway(self, index, pos, path, xlist, ylist, tailway = False, node_hops_max = 200,radius = 0.07,offset = 0):
		''' Finds the headway based on a given car and path. Also Finds
		tailway if tailway = True (obviously) '''
		#TODO: Below, maybe iterate every 5 nodes or so. Add a parameter and
		# tweak it based on the distance of each node. Remember, we only need to
		# poll every x amount of nodes, not every single one needs to be checked.
		#Radius matters for accuracy not the speed of running

		nodes = self.nodes[path]

		# Find our starting node
		start_node = None
		for i in range(len(nodes)):
			node = nodes[i]
			dist = math.hypot(pos[0] - node[0], pos[1] - node[1])
			if (math.fabs(dist) <= radius):
				start_node = i
				break
		if(start_node == None):
			bestGuess = 100000000000
			bestNode = 0
			for i in range(len(nodes)):
				node = nodes[i]
				dist = math.hypot(pos[0] - node[0], pos[1] - node[1])
				if (math.fabs(dist) < bestGuess):
					bestGuess = dist
					bestNode = node[i]

			rospy.logwarn("WARNING! No node found for car " + str(index) + "! Check parameters!")
			rospy.logwarn("Best Guess found was " + str(bestNode) + " at a distance of " + str(bestGuess) + " away.")
			rospy.logwarn("Pos: " + str(pos) + " path: " +str(path) + " xlist: " + str(xlist) + " ylist: " + str(ylist))
			#try:
		    #	start_node = bestNode[0]
			#except:
			#	start_node = bestNode
			return None

		# Iterate starting with our start node
		i = start_node + offset  # Start searching with the start node
		if (not tailway and i >= len(nodes)): # Restart at index 0 if you reach the end of the path
			i = i - len(nodes)
		if (tailway and i <= -1): # Restart at max index if you reach -1
			i = i + len(nodes)
		node_hops = 0   # Calc # of hops for accurate distance
		#loops = 0       # # of times we reset i=0
		while node_hops<=node_hops_max:
			try:
			    node = nodes[i] # Our current node
			except:
				print(i, len(nodes))
			for j in range(len(xlist)):
				if (j == index):
					continue    # Ignore the car we're using
				dist = math.hypot(xlist[j] - node[0], ylist[j] - node[1])
				if (math.fabs(dist) <= radius):
					return j, ((node_hops+offset)*self.speed + dist)
			if (tailway):
				i=i-1
			else:
				i=i+1 # Replace this with i=i+node_resolution to skip redundant nodes
			if (not tailway and i == len(nodes)): # Restart at index 0 if you reach the end of the path
				i = 0
				#loops=loops+1
			if (tailway and i == -1): # Restart at max index if you reach -1
				i = (len(nodes) - 1)
				#loops=loops+1
			node_hops=node_hops+1
		#rospy.logwarn("WARNING! No other car found during headway/tailway test! Is the car" + str(index) + "alone or far from other vehicles?")
		return index,10000
