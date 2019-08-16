import math		#import the math module
#coded by ishti
#last updated--> 08/29/2018
""" 
Input values -->
vmax --> maximum speed limit of the road (m/s)
v0 --> speed of the following vehicle (m/s)
v1 --> speed of the leading vehicle (m/s)
sn --> headway (m) [bumper to bumper distance]

Output values -->
a0 --> desired acceleration of the following vehicle (m/s2)
"""

def idm(sn, v0, v1, vmax = 30.0/25):
	###############################################################	
	#parameters
	s0 = 5.0/25		#minimum bumper to bumper gap (m)   [center to center gap, 2.0 + 5m for car length]
	a = 1.0/25		#max acceleration (m/s2)
	b = 1.5/25#3.0/25		#max comfortable deceleration (m/s2)
	delta = 4#2		#acceleration exponent
	th = 1#2			#time headway (s)
	##############################################################
	sn=max(sn,s0)
	#calculating fac0 and velocity difference
	fac0 = 2* math.sqrt(a*b)
	del_v = v0 - v1
	#calculating desired safe gap
	s_star = s0 + max(0, v0*th + (v0*del_v)/fac0)
	#desired acceleration on a free road
	free_acc = 1-(v0/vmax)**delta
	#braking decelration caused by the front car
	brake_decel =(s_star/sn)**2
	
	
	
	#calculating desired acceleration for the following vehicle [m/s2]
	a0 = a*(free_acc - brake_decel)
	
	return a0

	
