import math

def measureCircle(x,y,xRef,yRef,xCenter,yCenter,clockwise):
	#Returns the length along an arc
	#x,y is your position (floats)
	#xRef and yRef are coordinates of the reference point
	#xCenter, yCenter is the center of the circle (floats)
	#clockwise is a boolean for if you are measuring clockwise or counterclockwise
	radius = math.sqrt((xCenter-x)*(xCenter-x)+(yCenter-y)*(yCenter-y))
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
