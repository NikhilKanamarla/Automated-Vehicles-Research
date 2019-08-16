import math

def transformCoords(x,y,xDelta,yDelta,theta):
	#Returns new x and y coordinates in a tranformed coordinate system. For two cartesian coordinate systems
	#x,y are coordinates in original coordinate system
	#xDelta yDelta are the difference of the old origin and new origin in the original coordinate system
	#theta is the roatation between the two coordinate system compared to the original. Theta is in radians
	xNew = x - xDelta
	yNew = y - yDelta
	xNewF = math.cos(-theta)*xNew-math.sin(-theta)*yNew
	yNewF = math.sin(-theta)*xNew+math.cos(-theta)*yNew
	return xNewF,yNewF

def measureCircle(x,y,xRef,yRef,xCenter,yCenter,clockwise):
	#Returns the length along an arc
	#x,y is your position (floats)
	#xRef and yRef are coordinates of the reference point
	#xCenter, yCenter is the center of the circle (floats)
	#clockwise is a boolean for if you are measuring clockwise or counterclockwise
	radius = math.sqrt((xCenter-xRef)*(xCenter-xRef)+(yCenter-yRef)*(yCenter-yRef))
	#c = math.sqrt((x-xRef)*(x-xRef)+(y-yRef)*(y-yRef)) THIS IS OLD LAW OF COSINES STUFF, WILL DELETE WHEN CODE IS CONFIRMED TO WORK
	#theta = math.acos((radius*radius)*2-c*c)/(2*radius*radius))
	thetaRef = math.atan2(yRef-yCenter,xRef-xCenter)
        
	[xNew,yNew]=transformCoords(x,y,xCenter,yCenter,thetaRef)
	theta=math.atan2(yNew,xNew)
	
	if theta<0:
		theta=theta+2*math.pi
	if clockwise:
		theta=2*math.pi-theta
	length = radius*theta
	return length
