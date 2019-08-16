
#!/usr/bin/env python

import math

class Zone:
    def __init__(self, tx, ty, xi = True, yi = True):
        self.transX = tx
        self.transY = ty
        self.xIsLarger = xi
        self.yIsLarger = yi

    def checkX(self, x):
        if self.xIsLarger:
            if x>self.transX: return True
            else: return False
        else: 
            if x<self.transX: return True
            else: return False

    def checkY(self, y):
        if self.yIsLarger:
            if y>self.transY: return True
            else: return False
        else: 
            if y<self.transY: return True
            else: return False
    def ShouldTransit(self, x, y):
        if math.isnan(self.transX) and math.isnan(self.transY):
            return False
        elif math.isnan(self.transY):
            if self.checkX(x):
                return True
            else:
                return False
        elif math.isnan(self.transX):
            if self.checkY(y):
                return True
            else:
                return False
        else:
            if self.checkX(x) and self.checkY(y):
                return True
            else:
                return False

class ControlZone:
    def __init__(self, act_ty, cid, eid, tx, ty, xi = True, yi = True):
        self.border = Zone(tx,ty,xi,yi)
        self.res = (act_ty,cid,eid) 
#act_ty action type #act_ty: 0->Starting of a control zone 1->starting of a merge  2->(end of the control zone )(exit the merge)
#cid control id. for example cid for the mergsim is 0 , and for intersection scenario, since we have two cz in intersim is 1 and 2(0 is already used for mergsim) . 
#eid id of groups which are entering to the cz(Control type)
# if we have two action types in a same segment, it should be writen in order ex.-> 2 1 0

    def CheckZone(self, x, y):
        if self.border.ShouldTransit(x,y):
            return self.res
        else:
            return None

class Segment:

    def __init__(self, sid, tx, ty, xi = True, yi = True):
        self.track = Zone(tx,ty,xi,yi)
        self.segId = sid
        self.cZones = [] #WARNING: SEQUENCE MATERS

    def AppendControlBorderByRef(self, se):
        self.cZones.append(se)

    def AppendControlBorder(self, act_ty, cid, eid, tx, ty, xi = True, yi = True):
        self.cZones.append(ControlZone(act_ty,cid,eid,tx,ty,xi,yi))

    def ShouldTransit(self, x, y):
        return self.track.ShouldTransit(x,y)

    def CheckControl(self, x, y):
        for c in self.cZones:
            res = c.CheckZone(x,y)
            if res is not None:
                return res
        return None

class Path:

    def __init__(self):
        self.segments = []
        self.index = 0

    def AppendSegmentByRef(self, se):
        self.segments.append(se)

    def AppendSegment(self, sid, tx, ty, xi, yi): #adds a new segment with passed-in dimensions
        self.segments.append(Segment(sid,tx,ty,xi,yi))
    
    def CheckControl(self, x, y):
        return self.segments[self.index].CheckControl(x,y)

    def GetSegmentID(self, x, y):
        if len(self.segments) == 0:
            return -1
        else:
            if self.segments[self.index].ShouldTransit(x, y):
                self.index = self.index+1
                if self.index>=len(self.segments):
                    self.index = 0
                return self.segments[self.index].segId
            else:
                return self.segments[self.index].segId

    def GetSegmentIDNoCheck(self):
        if len(self.segments) == 0:
            return -1
        else:
            return self.segments[self.index].segId#segId is the source id which is defined in map builder

def GetDefaultPath(index):
    res = Path()
    if index == 0:#cars start on far arc farthest from the main frame
        res.AppendSegment(2,-1.61,0.2042,True,True) #(transitionfrom,xhalfplane,yhalfplane,true if you want > x value,true if you want > y value)
        res.AppendSegment(3,-0.8744,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(0,0,0,-1.02885654,float('nan'),True,True)

        res.AppendSegment(4,1.1263,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(2,0,0,0.645411,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(1,0,0,0.295411,float('nan'),True,True)

        res.AppendSegment(5,float('nan'),0.435,True,False)
        res.AppendSegment(14,2.2352,float('nan'),True,True)
        res.AppendSegment(15,float('nan'),1.2,True,True)
        res.AppendSegment(6,2.26,float('nan'),False,True)
	res.AppendSegment(0,-0.9506,float('nan'),False,True)
        res.AppendSegment(1,-2.276,float('nan'),False,True)
	return res
    if index == 2:#cars start on arc closest to the mainframe
        res.AppendSegment(14,2.2352,float('nan'),True,True)
        res.AppendSegment(15,float('nan'),1.2,True,True)
        res.AppendSegment(6,2.26,float('nan'),False,True)
	res.AppendSegment(0,-0.9506,float('nan'),False,True)
        res.AppendSegment(1,-2.276,float('nan'),False,True)
	res.AppendSegment(2,-1.61,0.2042,True,True) #(transitionfrom,xhalfplane,yhalfplane,true if you want > x value,true if you want > y value)
        res.AppendSegment(3,-0.8744,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(0,0,0,-1.0715,float('nan'),True,True)

        res.AppendSegment(4,1.1263,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(2,0,0,0.603929,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(1,0,0,0.295411,float('nan'),True,True)

        res.AppendSegment(5,float('nan'),0.435,True,False)

	return res
    if index == 3:#cars start on the bottom straight
	res.AppendSegment(0,-0.9506,float('nan'),False,True)
        res.AppendSegment(1,-2.276,float('nan'),False,True)
        res.AppendSegment(2,-1.61,0.2042,True,True) #(transitionfrom,xhalfplane,yhalfplane,true if you want > x value,true if you want > y value)
        res.AppendSegment(3,-0.8744,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(0,0,0,-1.0715,float('nan'),True,True)

        res.AppendSegment(4,1.1263,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(2,0,0,0.603929,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(1,0,0,0.295411,float('nan'),True,True)

        res.AppendSegment(5,float('nan'),0.435,True,False)
        res.AppendSegment(14,2.2352,float('nan'),True,True)
        res.AppendSegment(15,float('nan'),1.2,True,True)
        res.AppendSegment(6,2.26,float('nan'),False,True)
	return res
    if index == 4:#cars start on arc immediately after bottom straight
        res.AppendSegment(1,-2.276,float('nan'),False,True)
        res.AppendSegment(2,-1.61,0.2042,True,True) #(transitionfrom,xhalfplane,yhalfplane,true if you want > x value,true if you want > y value)
        res.AppendSegment(3,-0.8744,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(0,0,0,-1.0715,float('nan'),True,True)

        res.AppendSegment(4,1.1263,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(2,0,0,0.603929,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(1,0,0,0.295411,float('nan'),True,True)

        res.AppendSegment(5,float('nan'),0.435,True,False)
        res.AppendSegment(14,2.2352,float('nan'),True,True)
        res.AppendSegment(15,float('nan'),1.2,True,True)
        res.AppendSegment(6,2.26,float('nan'),False,True)
	res.AppendSegment(0,-0.9506,float('nan'),False,True)
        return res
####################################################SHORTER PATHS#######################################################################
    elif index == 1:#Start on super long straight for merging demonstration
        res.AppendSegment(9,float('nan'),0.3010,True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(0,0,1,float('nan'),-0.64902705,True,True)

        res.AppendSegment(10,0.6369,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(1,0,1,0.353632,float('nan'),True,True)

        res.AppendSegment(4,1.1963,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(2,0,1,0.645411,float('nan'),True,True)

        res.AppendSegment(5,float('nan'),0.5,True,False)
        res.AppendSegment(14,1.997,float('nan'),True,True)
	res.AppendSegment(11,float('nan'),-0.2032,True,False) #added this arc back in
        res.AppendSegment(12,float('nan'),-0.8382+0.02,True,False) #-0.854
        res.AppendSegment(13,2.03,float('nan'),False,True) #2.00304
        res.AppendSegment(7,0.381+0.1,float('nan'),False,True) #0.46
        res.AppendSegment(8,float('nan'),-0.8382+0.02,True,True)
        return res
    elif index == 5:#Arc after merging area
        res.AppendSegment(5,float('nan'),0.435,True,False)
        res.AppendSegment(14,1.997,float('nan'),True,True)
	res.AppendSegment(11,float('nan'),-0.2032,True,False) #added this arc back in
        res.AppendSegment(12,float('nan'),-0.8382+0.02,True,False) #-0.854
        res.AppendSegment(13,2.007,float('nan'),False,True) #2.00304
        res.AppendSegment(7,0.381+0.02,float('nan'),False,True) #0.46
        res.AppendSegment(8,float('nan'),-0.8382+0.02,True,True)
	res.AppendSegment(9,float('nan'),0.3010,True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(0,0,1,float('nan'),-0.698549,True,True)

        res.AppendSegment(10,0.6369,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(1,0,1,0.353632,float('nan'),True,True)

        res.AppendSegment(4,1.1963,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(2,0,1,0.603929,float('nan'),True,True)

        return res
    elif index == 6:#close loop arc
        res.AppendSegment(14,1.997,float('nan'),True,True)
	res.AppendSegment(11,float('nan'),-0.2032,True,False) #added this arc back in
        res.AppendSegment(12,float('nan'),-0.8382+0.02,True,False) #-0.854
        res.AppendSegment(13,2.007,float('nan'),False,True) #2.00304
        res.AppendSegment(7,0.381+0.02,float('nan'),False,True) #0.46
        res.AppendSegment(8,float('nan'),-0.8382+0.02,True,True)
        res.AppendSegment(9,float('nan'),0.3010,True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(0,0,1,float('nan'),-0.698549,True,True)

        res.AppendSegment(10,0.6369,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(1,0,1,0.353632,float('nan'),True,True)

        res.AppendSegment(4,1.1963,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(2,0,1,0.603929,float('nan'),True,True)

        res.AppendSegment(5,float('nan'),0.435,True,False)
        return res
    elif index == 7:#close short after roundabout
        res.AppendSegment(12,float('nan'),-0.8382+0.02,True,False) #-0.854
        '''res.AppendSegment(5,float('nan'),0.435,True,False)
        res.AppendSegment(6,2.05,float('nan'),True,True)
        res.AppendSegment(12,float('nan'),-0.854,True,False) #-0.854
	res.AppendSegment(11,3.048-1.47,float('nan'),True,True) #added this arc back in'''
        res.AppendSegment(13,2.007,float('nan'),False,True) #2.00304
        res.AppendSegment(7,0.381+0.02,float('nan'),False,True) #0.46
        res.AppendSegment(8,float('nan'),-0.8382+0.08,True,True)
        res.AppendSegment(9,float('nan'),0.3010,True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(0,0,1,float('nan'),-0.698549,True,True)

        res.AppendSegment(10,0.6369,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(1,0,1,0.353632,float('nan'),True,True) #control zone

        res.AppendSegment(4,1.1963,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(2,0,1,0.603929,float('nan'),True,True)

        res.AppendSegment(5,float('nan'),0.435,True,False)
        res.AppendSegment(14,1.997,float('nan'),True,True)
	res.AppendSegment(11,float('nan'),-0.2032,True,False) #added this arc back in
        return res

    elif index == 8:#Start on super long straight for merging demonstration

        res.AppendSegment(7,0.381+0.1,float('nan'),False,True) #0.46
        res.AppendSegment(8,float('nan'),-0.8382+0.02,True,True)
        res.AppendSegment(9,float('nan'),0.3010,True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(0,0,1,float('nan'),-0.64902705,True,True)
        res.AppendSegment(10,0.6369,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(1,0,1,0.353632,float('nan'),True,True)
        res.AppendSegment(4,1.1963,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(2,0,1,0.645411,float('nan'),True,True)
        res.AppendSegment(5,float('nan'),0.5,True,False)
        res.AppendSegment(14,1.997,float('nan'),True,True)
	res.AppendSegment(11,float('nan'),-0.2032,True,False) #added this arc back in
        res.AppendSegment(12,float('nan'),-0.8382+0.02,True,False) #-0.854
        res.AppendSegment(13,2.03,float('nan'),False,True) #2.00304
        return res



#Testing B
    elif index == 9:#for test 3 
        res.AppendSegment(16,-1.8,float('nan'),False,True) #
        res.AppendSegment(17,-0.5,float('nan'),True,True) #
        return res
####################################################LEFT TO RIGHT INTERSECTION TEST #######################################################################
    elif index == 10:#for test 3
        res.AppendSegment(34,float('nan'),-3.1496,False,False)
        res.AppendSegment(37,2.159,float('nan'),False,False)
        res.AppendSegment(18,0.9398,float('nan'),False,True) 
        res.segments[len(res.segments)-1].AppendControlBorder(0,0,0,1.728
,float('nan'),False,False)
        res.AppendSegment(19,0.2286,float('nan'),False,True) 
        res.AppendSegment(20,-0.2286,float('nan'),False,True)
        res.segments[len(res.segments)-1].AppendControlBorder(1,0,0,0.228
,float('nan'),False,False)
        res.AppendSegment(21,-1.75,float('nan'),False,True)
        res.segments[len(res.segments)-1].AppendControlBorder(2,0,0,-0.232
,float('nan'),False,False)
        res.AppendSegment(38,float('nan'),-3.20,False,True)#A23
        res.AppendSegment(31,float('nan'),-1.5,False,True)
        res.AppendSegment(32,-1.8034,float('nan'),True,True)
        res.AppendSegment(33,-1.5384,float('nan'),True,True)
        res.AppendSegment(22,-0.3809,float('nan'),True,True) 
        res.AppendSegment(35,2,float('nan'),True,True)
        res.AppendSegment(36,float('nan'),-1.6,False,False)    

        return res
######################################## the top loop INTERSECTION TEST
    elif index == 11:#for test 3 
        res.AppendSegment(22,-0.3809,float('nan'),True,True) 
        res.AppendSegment(23,float('nan'),-1.6,False,False)
        res.AppendSegment(24,float('nan'),-2.6049,False,False)
        res.segments[len(res.segments)-1].AppendControlBorder(0,0,0,float('nan'),-1.646,True,False)
        res.AppendSegment(25,float('nan'),-3.1496,False,False) 
        res.AppendSegment(26,float('nan'),-3.55,False,False)
        res.segments[len(res.segments)-1].AppendControlBorder(1,0,0,float('nan'),-3.146,True,False)#need  to be double checked
        res.AppendSegment(27,float('nan'),-3.8,False,False)
        res.segments[len(res.segments)-1].AppendControlBorder(2,0,0,float('nan'),-3.606,True,False)
        res.AppendSegment(28,-0.2296,float('nan'),False,False)
        res.AppendSegment(29,-1.7,float('nan'),False,False)
        res.AppendSegment(30,float('nan'),-3.9878,False,True)#A9
        res.AppendSegment(31,float('nan'),-1.5,False,True)#Transition from 31 to 32(which is arc)
        res.AppendSegment(32,-1.8034,float('nan'),True,True)
        res.AppendSegment(33,-1.5384,float('nan'),True,True)

        return res
########################################
    elif index == 12:#for Intersim Left to right S34
        res.AppendSegment(34,float('nan'),-3.1496,False,False)
        res.AppendSegment(37,2.159,float('nan'),False,False)
        res.AppendSegment(18,0.9398,float('nan'),False,True) 
        res.segments[len(res.segments)-1].AppendControlBorder(0,1,0,1.728,float('nan'),False,False)
        res.AppendSegment(19,0.2286,float('nan'),False,True) 
        res.AppendSegment(20,-0.2286,float('nan'),False,True)
        res.segments[len(res.segments)-1].AppendControlBorder(1,1,0,0.228,float('nan'),False,False)
        res.AppendSegment(21,-1.75,float('nan'),False,True)
        res.segments[len(res.segments)-1].AppendControlBorder(0,2,0,-0.3034,float('nan'),False,False)
        res.segments[len(res.segments)-1].AppendControlBorder(2,1,0,-0.252,float('nan'),False,False)

        res.AppendSegment(38,float('nan'),-3.20,False,True)#A23
        res.segments[len(res.segments)-1].AppendControlBorder(1,2,0,-1.5826,float('nan'),False,False)
        res.AppendSegment(31,float('nan'),-1.5,False,True)
        res.segments[len(res.segments)-1].AppendControlBorder(2,2,0,float('nan'),-3.139,False,True)
        res.AppendSegment(32,-1.8034,float('nan'),True,True)
        res.AppendSegment(33,-1.5384,float('nan'),True,True)
        res.AppendSegment(22,-0.3809,float('nan'),True,True) 
        res.AppendSegment(35,2,float('nan'),True,True)
        res.AppendSegment(36,float('nan'),-1.6,False,False)    

        return res
######################################## the top loop INTERSECTION TEST

####Starting at S68
    elif index == 13:#For Intersim down to up
        res.AppendSegment(22,-0.3809,float('nan'),True,True) 
        res.AppendSegment(23,float('nan'),-1.6,False,False)
        res.AppendSegment(24,float('nan'),-2.6049,False,False)
        res.segments[len(res.segments)-1].AppendControlBorder(0,1,1,float('nan'),-1.646,True,False)
        res.AppendSegment(25,float('nan'),-3.1496,False,False) 
        res.AppendSegment(26,float('nan'),-3.55,False,False)
        res.segments[len(res.segments)-1].AppendControlBorder(1,1,1,float('nan'),-3.146,True,False)
        res.AppendSegment(27,float('nan'),-3.8,False,False)
        res.segments[len(res.segments)-1].AppendControlBorder(2,1,1,float('nan'),-3.626,True,False)
        res.AppendSegment(28,-0.2296,float('nan'),False,False)
        res.AppendSegment(29,-1.7,float('nan'),False,False)
        res.segments[len(res.segments)-1].AppendControlBorder(0,2,1,-0.8886,float('nan'),False,False)
        res.AppendSegment(30,float('nan'),-3.9878,False,True)#A9
        res.AppendSegment(31,float('nan'),-1.5,False,True)#Transition from 31 to 32(which is arc)
        res.segments[len(res.segments)-1].AppendControlBorder(1,2,1,float('nan'),-3.65,True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(2,2,1,float('nan'),-3.139,True,True)
        res.AppendSegment(32,-1.8034,float('nan'),True,True)
        res.AppendSegment(33,-1.5384,float('nan'),True,True)

        return res

####Starting at S6
    elif index == 14:#For Intersim down to up

        res.AppendSegment(29,-1.7,float('nan'),False,False)#-1.7
        res.segments[len(res.segments)-1].AppendControlBorder(0,2,1,-0.8886,float('nan'),False,False)
        res.AppendSegment(30,float('nan'),-3.9878,False,True)#A9
        res.AppendSegment(31,float('nan'),-1.5,False,True)#Transition from 31 to 32(which is arc)
        res.segments[len(res.segments)-1].AppendControlBorder(1,2,1,float('nan'),-3.65,True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(2,2,1,float('nan'),-3.139,True,True)
        res.AppendSegment(32,-1.8034,float('nan'),True,True)
        res.AppendSegment(33,-1.5384,float('nan'),True,True)
        res.AppendSegment(22,-0.3809,float('nan'),True,True) 
        res.AppendSegment(23,float('nan'),-1.6,False,False)
        res.AppendSegment(24,float('nan'),-2.6049,False,False)
        res.segments[len(res.segments)-1].AppendControlBorder(0,1,1,float('nan'),-1.646,True,False)
        res.AppendSegment(25,float('nan'),-3.1496,False,False) 
        res.AppendSegment(26,float('nan'),-3.55,False,False)
        res.segments[len(res.segments)-1].AppendControlBorder(1,1,1,float('nan'),-3.146,True,False)
        res.AppendSegment(27,float('nan'),-3.8,False,False)
        res.segments[len(res.segments)-1].AppendControlBorder(2,1,1,float('nan'),-3.626,True,False)
        res.AppendSegment(28,-0.2296,float('nan'),False,False)

        return res

    elif index == 15:#for Intersim Left to right  S62 S58
        res.AppendSegment(35,2,float('nan'),True,True)
        res.AppendSegment(36,float('nan'),-1.6,False,False)    
        res.AppendSegment(34,float('nan'),-3.1496,False,False)
        res.AppendSegment(37,2.159,float('nan'),False,False)
        res.AppendSegment(18,0.9398,float('nan'),False,True) 
        res.AppendSegment(19,0.2286,float('nan'),False,True) 
        res.AppendSegment(20,-0.2286,float('nan'),False,True)
        res.AppendSegment(21,-1.75,float('nan'),False,True)
        res.AppendSegment(38,float('nan'),-3.20,False,True)#A23
        res.AppendSegment(31,float('nan'),-1.5,False,True)
        res.AppendSegment(32,-1.8034,float('nan'),True,True)
        res.AppendSegment(33,-1.5384,float('nan'),True,True)
        res.AppendSegment(22,-0.3809,float('nan'),True,True) 

        return res
    elif index == 16:#for Intersim Left to right  S10
        res.AppendSegment(31,float('nan'),-1.5,False,True)
        res.AppendSegment(32,-1.8034,float('nan'),True,True)
        res.AppendSegment(33,-1.5384,float('nan'),True,True)
        res.AppendSegment(22,-0.3809,float('nan'),True,True) 
        res.AppendSegment(35,2,float('nan'),True,True)
        res.AppendSegment(36,float('nan'),-1.6,False,False)    
        res.AppendSegment(34,float('nan'),-3.1496,False,False)
        res.AppendSegment(37,2.159,float('nan'),False,False)
        res.AppendSegment(18,0.9398,float('nan'),False,True) 
        res.AppendSegment(19,0.2286,float('nan'),False,True) 
        res.AppendSegment(20,-0.2286,float('nan'),False,True)
        res.AppendSegment(21,-1.75,float('nan'),False,True)
        res.AppendSegment(38,float('nan'),-3.20,False,True)#A23



        return res

    elif index == 17:#for Intersim Left to right S34
        res.AppendSegment(34,float('nan'),-3.1496,False,False)
        res.AppendSegment(37,2.159,float('nan'),False,False)
        res.AppendSegment(18,0.9398,float('nan'),False,True) 
        res.AppendSegment(19,0.2286,float('nan'),False,True) 
        res.AppendSegment(20,-0.2286,float('nan'),False,True)
        res.AppendSegment(21,-1.75,float('nan'),False,True)
        res.AppendSegment(38,float('nan'),-3.20,False,True) #A23
        res.AppendSegment(31,float('nan'),-1.5,False,True)
        res.AppendSegment(32,-1.8034,float('nan'),True,True)
        res.AppendSegment(33,-1.5384,float('nan'),True,True)
        res.AppendSegment(22,-0.3809,float('nan'),True,True) 
        res.AppendSegment(35,2,float('nan'),True,True)
        res.AppendSegment(36,float('nan'),-1.6,False,False)    

        return res

    elif index == 18:#for Intersim Left to right S34

        res.AppendSegment(18,0.9398,float('nan'),False,True) 
        res.AppendSegment(19,0.2286,float('nan'),False,True) 
        res.AppendSegment(20,-0.2286,float('nan'),False,True)
        res.AppendSegment(21,-1.75,float('nan'),False,True)
        res.AppendSegment(38,float('nan'),-3.20,False,True)#A23
        res.AppendSegment(31,float('nan'),-1.5,False,True)
        res.AppendSegment(32,-1.8034,float('nan'),True,True)
        res.AppendSegment(33,-1.5384,float('nan'),True,True)
        res.AppendSegment(22,-0.3809,float('nan'),True,True) 
        res.AppendSegment(35,2,float('nan'),True,True)
        res.AppendSegment(36,float('nan'),-1.6,False,False)
        res.AppendSegment(34,float('nan'),-3.1496,False,False)
        res.AppendSegment(37,2.159,float('nan'),False,False)   

        return res
    elif index == 19:#for Intersim Left to right S34
        res.AppendSegment(21,-1.75,float('nan'),False,True)
        res.AppendSegment(38,float('nan'),-3.20,False,True)#A23
        res.AppendSegment(31,float('nan'),-1.5,False,True)
        res.AppendSegment(32,-1.8034,float('nan'),True,True)
        res.AppendSegment(33,-1.5384,float('nan'),True,True)
        res.AppendSegment(22,-0.3809,float('nan'),True,True) 
        res.AppendSegment(35,2,float('nan'),True,True)
        res.AppendSegment(36,float('nan'),-1.6,False,False)
        res.AppendSegment(34,float('nan'),-3.1496,False,False)
        res.AppendSegment(37,2.159,float('nan'),False,False)
        res.AppendSegment(18,0.9398,float('nan'),False,True) 
        res.AppendSegment(19,0.2286,float('nan'),False,True) 
        res.AppendSegment(20,-0.2286,float('nan'),False,True)   

        return res
    elif index == 20:#downloop on arc 105 101 100
        res.AppendSegment(2,-1.61,0.2042,True,True) #(segement transition from,posotion of xhalfplane,postion ofyhalfplane,true if you want > x half plane value,true if you want > y half plane value)
        res.AppendSegment(3,-0.8744,float('nan'),True,True)
        res.AppendSegment(4,1.1263,float('nan'),True,True)
        res.AppendSegment(5,float('nan'),0.435,True,False)
        res.AppendSegment(14,2.2352,float('nan'),True,True)
        res.AppendSegment(15,float('nan'),1.2,True,True)
        res.AppendSegment(6,2.26,float('nan'),False,True)
	res.AppendSegment(0,-0.9506,float('nan'),False,True)
        res.AppendSegment(1,-2.276,float('nan'),False,True)
	return res

    elif index == 21:#cars start on the bottom straight
	res.AppendSegment(0,-0.9506,float('nan'),False,True)
        res.AppendSegment(1,-2.276,float('nan'),False,True)
        res.AppendSegment(2,-1.61,0.2042,True,True)
        res.AppendSegment(3,-0.8744,float('nan'),True,True)
        res.AppendSegment(4,1.1263,float('nan'),True,True)
        res.AppendSegment(5,float('nan'),0.435,True,False)
        res.AppendSegment(14,2.2352,float('nan'),True,True)
        res.AppendSegment(15,float('nan'),1.2,True,True)
        res.AppendSegment(6,2.26,float('nan'),False,True)
	return res

    elif index == 22:#cars start on the bottom straight

        res.AppendSegment(4,1.1263,float('nan'),True,True)
        res.AppendSegment(5,float('nan'),0.435,True,False)
        res.AppendSegment(14,2.2352,float('nan'),True,True)
        res.AppendSegment(15,float('nan'),1.2,True,True)
        res.AppendSegment(6,2.26,float('nan'),False,True)
	res.AppendSegment(0,-0.9506,float('nan'),False,True)
        res.AppendSegment(1,-2.276,float('nan'),False,True)
        res.AppendSegment(2,-1.61,0.2042,True,True)
        res.AppendSegment(3,-0.8744,float('nan'),True,True)

	return res

          #index 23 and 24 are built using the new map builder 
    elif index == 23: #bottom left roundabout 
	#A87, A84, A81, A89 source numbers will be different than this

   
        res.AppendSegment(179,float('nan'),0.424,True,True) #a87
        res.AppendSegment(176,2,float('nan'),True,True) #a84
        res.AppendSegment(173,float('nan'),0.254,True,True) #a81
        res.AppendSegment(176,2.238,float('nan'),True,True) #a89
	return res


    elif index == 24: #goes from bottom left roundabout through secret pathway
	#A87, A86, S88, A53, S61, A78, S84, A93, A91
	res.AppendSegment(179,float('nan'),0.424,True,True) #a87
	res.AppendSegment(178,2,float('nan'),True,True) #a86
	res.AppendSegment(84,float('nan'),-0.206,True,True) #S88
	res.AppendSegment(145,float('nan'),-0.846,True,True) #a53
	res.AppendSegment(57,2.008, float('nan'),True,True) #S61
	res.AppendSegment(170,1.168,float('nan'),True,True) #a78
	res.AppendSegment(80,float('nan'),-0.536,True,True) #s84
	res.AppendSegment(185,float('nan'),0.304,True,True) #a93
	res.AppendSegment(183,1.118,float('nan'),True,True) #a91

	return res

    elif index == 25: #Test new version of mapbuilder
	#res.AppendSegment(4,1.1263,float('nan'),True,True) #comment by Nikhil for testing
	res.AppendSegment(0,1.7,float('nan'),True,True)

        return res

    elif index == 26: #Path for leader straight
	
	res.AppendSegment(46,2.01,float('nan'),False,True)#start
	res.AppendSegment(41,0.38,float('nan'),False,True)#S60
	res.AppendSegment(42,-0.38,float('nan'),False,True)#S71
	res.AppendSegment(43,-1.54,float('nan'),False,True)#S72
	res.AppendSegment(44,-1.81,float('nan'),False,True)#S75
	res.AppendSegment(45,-2,float('nan'),False,True)#Stop

        return res
    elif index == 27: #Path for just going around the right roundabout  (OLD MAPBUILDER)

	res.AppendSegment(2,-1.61,0.2042,True,True)#A105-A101-A100
	res.AppendSegment(47,-2.26,.917,False,False)#A108


#########################

	res.AppendSegment(200,2.01,float('nan'),False,True)
	res.AppendSegment(197,0.38,float('nan'),False,True)
	res.AppendSegment(195,2.01,float('nan'),False,True)
	res.AppendSegment(75,0.38,float('nan'),False,True)
	res.AppendSegment(72,2.01,float('nan'),False,True)
	res.AppendSegment(40,0.38,float('nan'),False,True)

        return res





#############################################UCBerkeley##########################################################
#it is designed for Auto mapbuilder#
##PATH1## A102-A100-A108-A105-A103-S79-S76-S44-A43-A41-S37-A6-A5-S10-S11-S34-S40-S45-S77-S78
    elif index == 28: 
	res.AppendSegment(74,float('nan'),-0.69,True,True)
	res.AppendSegment(194,-1.7,float('nan'),True,True)#y=-0.091
	res.AppendSegment(192,float('nan'),0.35,True,True)
	res.AppendSegment(200,-2.07,float('nan'),False,False)#-2.336,0.884
	res.AppendSegment(197,-2.329,-0.187,True,False)
	res.AppendSegment(195,float('nan'),-0.762,False,False)
	res.AppendSegment(75,float('nan'),-1.067+0.076,False,False)
	res.AppendSegment(72,float('nan'),-1.524+0.076,False,False)
	res.AppendSegment(40,float('nan'),-2.083,False,False)
	res.AppendSegment(135,-2.337,float('nan'),False,False)
	res.AppendSegment(133,float('nan'),-2.845,False,False)
	res.AppendSegment(33,float('nan'),4.14,False,False)
	res.AppendSegment(98,-2.26,float('nan'),True,False)#A6
        res.segments[len(res.segments)-1].AppendControlBorder(0,4,1,float('nan'),-3.986,True,False)
	res.AppendSegment(97,float('nan'),-3.916,True,True)
	res.AppendSegment(9,float('nan'),-3.531,True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(1,4,1,float('nan'),-3.7,True,True)
	res.AppendSegment(10,float('nan'),-3.073,True,True)
	res.AppendSegment(30,float('nan'),-2.159,True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(2,4,1,float('nan'),-3.1,True,True)
	res.AppendSegment(36,float('nan'),-1.93,True,True)
	res.AppendSegment(41,float('nan'),-1.372,True,True)
	res.AppendSegment(73,float('nan'),-0.92,True,True)


        return res


#Path2
    elif index == 29: 
	res.AppendSegment(74,float('nan'),-0.69,True,True)#S78
	res.AppendSegment(194,-1.7,float('nan'),True,True)#A102
	res.AppendSegment(192,float('nan'),0.35-0.076,True,True)#A100
	res.AppendSegment(191,-0.872,float('nan'),True,True)#A99
	res.AppendSegment(91,-0.872+0.16,float('nan'),True,True)#S95
	res.AppendSegment(189,float('nan'),0.229,True,False)#A97
	res.AppendSegment(79,float('nan'),-0.914,True,False)#S83
	res.AppendSegment(63,float('nan'),-1.676,True,False)#S67
	res.AppendSegment(42,float('nan'),-2.159+0.076,True,False)#S46
	res.AppendSegment(209,float('nan'),-2.327+0.076,True,False)#A117
	res.AppendSegment(208,float('nan'),-2.681,True,False)#A116
	res.AppendSegment(27,float('nan'),-3.226+0.076,True,False)#S31
	res.AppendSegment(117,-0.305,float('nan'),False,False)#A25
	res.AppendSegment(13,-1.88+.076,float('nan'),False,False)#S14
        res.segments[len(res.segments)-1].AppendControlBorder(1,4,2,-1.75,float('nan'),False,False)
        res.segments[len(res.segments)-1].AppendControlBorder(0,4,2,-0.3034,float('nan'),False,False)
	res.AppendSegment(115,float('nan'),-3.073,False,True)#A23
	res.AppendSegment(30,float('nan'),-2.159,True,True)#S34
        res.segments[len(res.segments)-1].AppendControlBorder(2,4,2,float('nan'),-3.1,True,True)
	res.AppendSegment(36,float('nan'),-1.93,True,True)#S40
	res.AppendSegment(41,float('nan'),-1.372,True,True)#S45
	res.AppendSegment(73,float('nan'),-0.92,True,True)#S77


        return res
#path3
    elif index == 30: 
	res.AppendSegment(90,-0.951+0.076,float('nan'),False,True)#S94
	res.AppendSegment(198,-2.336+0.076,0.884+0.076,False,False)#A106
	res.AppendSegment(197,-2.329-0.076,0.278,True,False)#A105
	res.AppendSegment(193,-1.617,-0.091,True,True)#A101
	res.AppendSegment(192,float('nan'),0.35-0.076,True,True)#A100
	res.AppendSegment(191,-0.872,float('nan'),True,True)#A99
        res.segments[len(res.segments)-1].AppendControlBorder(0,3,0,-1.02885654,float('nan'),True,True)
	res.AppendSegment(91,-0.872+0.16,float('nan'),True,True)#S95
	res.AppendSegment(87,0.713-0.076,float('nan'),True,True)#S91
        res.segments[len(res.segments)-1].AppendControlBorder(2,3,0,0.603929,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(1,3,0,0.295411,float('nan'),True,True)
	res.AppendSegment(85,1.196-0.076,float('nan'),True,True)#S89
	res.AppendSegment(184,1.685-0.076,0.347+0.076,True,False)#A92
	res.AppendSegment(179,2.065-0.076,float('nan'),True,False)#A87
	res.AppendSegment(176,2.73,0.359-0.076,True,True)#A84
	res.AppendSegment(173,2.159+0.0716,float('nan'),False,True)#A81


        return res
#path4
    elif index == 31: 
	res.AppendSegment(90,-0.951+0.076,float('nan'),False,True)#S94
	res.AppendSegment(198,-2.336+0.076,0.884+0.076,False,False)#A106
	res.AppendSegment(197,-2.329-0.076,0.278,True,False)#A105
	res.AppendSegment(195,float('nan'),-0.762,False,False)#A103
	res.AppendSegment(75,float('nan'),-1.067+0.076,False,False)#S79
	res.AppendSegment(72,float('nan'),-1.524+0.076,False,False)#S76
	res.AppendSegment(40,float('nan'),-1.532,False,False)#S44
	res.AppendSegment(34,float('nan'),-2.54+.076,False,False)#S38
        res.segments[len(res.segments)-1].AppendControlBorder(0,4,0,float('nan'),-2.1,True,False)
	res.AppendSegment(31,float('nan'),-3.226+.076,False,False)#S35
        res.segments[len(res.segments)-1].AppendControlBorder(1,4,0,float('nan'),-3.1,True,False)
	res.AppendSegment(114,-1.727-0.076,float('nan'),True,False)#A22
	res.AppendSegment(12,-0.152-0.076,float('nan'),True,False)#S13
        res.segments[len(res.segments)-1].AppendControlBorder(2,4,0,-1.75,float('nan'),True,False)
	res.AppendSegment(116,float('nan'),-3.073-0.076,True,True)#A24
	res.AppendSegment(26,float('nan'),-2.529-0.076,True,True)#S30
	res.AppendSegment(207,float('nan'),-2.192-0.076,True,True)#A115
	res.AppendSegment(206,float('nan'),-2.007-0.076,True,True)#A114
	res.AppendSegment(44,float('nan'),-1.524-0.076,True,True)#S48
	res.AppendSegment(61,float('nan'),-0.762-0.076,True,True)#S65
	res.AppendSegment(76,float('nan'),0.381-0.076,True,True)#S80
        res.segments[len(res.segments)-1].AppendControlBorder(0,3,1,float('nan'),-0.64902705,True,True)
	res.AppendSegment(186,0.713,float('nan'),True,True)#S94
        res.segments[len(res.segments)-1].AppendControlBorder(2,3,1,0.645411,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(1,3,1,0.353632,float('nan'),True,True)
	res.AppendSegment(85,1.196-0.076,float('nan'),True,True)#S89
	res.AppendSegment(184,1.685-0.076,0.347+0.076,True,False)#A92
	res.AppendSegment(179,2.065-0.076,float('nan'),True,False)#A87
	res.AppendSegment(176,2.73,0.359-0.076,True,True)#A84
	res.AppendSegment(173,2.159+0.0716,float('nan'),False,True)#A81
        return res
##PATH1-2
    elif index == 32: 
	res.AppendSegment(41,float('nan'),-1.372,True,True)
	res.AppendSegment(73,float('nan'),-0.92,True,True)
	res.AppendSegment(74,float('nan'),-0.69,True,True)
	res.AppendSegment(194,-1.7,float('nan'),True,True)
	res.AppendSegment(192,float('nan'),0.35,True,True)
	res.AppendSegment(200,-2.07,float('nan'),False,False)
	res.AppendSegment(197,-2.329-0.076,0.278,True,False)#-0.187+0.076
	res.AppendSegment(195,float('nan'),-0.762,False,False)
	res.AppendSegment(75,float('nan'),-1.067+0.076,False,False)
	res.AppendSegment(72,float('nan'),-1.524+0.076,False,False)
	res.AppendSegment(40,float('nan'),-2.083,False,False)
	res.AppendSegment(135,-2.337,float('nan'),False,False)
	res.AppendSegment(133,float('nan'),-2.845,False,False)
	res.AppendSegment(33,float('nan'),4.14,False,False)
	res.AppendSegment(98,-2.26,float('nan'),True,False)
        res.segments[len(res.segments)-1].AppendControlBorder(0,4,1,float('nan'),-3.986,True,False)
	res.AppendSegment(97,float('nan'),-3.916,True,True)
	res.AppendSegment(9,float('nan'),-3.531,True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(1,4,1,float('nan'),-3.7,True,True)
	res.AppendSegment(10,float('nan'),-3.073,True,True)
	res.AppendSegment(30,float('nan'),-2.159,True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(2,4,1,float('nan'),-3.1,True,True)
	res.AppendSegment(36,float('nan'),-1.93,True,True)



        return res
##PATH2-2
    elif index == 33: 

	res.AppendSegment(41,float('nan'),-1.372,True,True)#S45
	res.AppendSegment(73,float('nan'),-0.92,True,True)#S77
	res.AppendSegment(74,float('nan'),-0.69,True,True)#S78
	res.AppendSegment(194,-1.7,float('nan'),True,True)#A102
	res.AppendSegment(192,float('nan'),0.35-.076,True,True)#A100
	res.AppendSegment(191,-0.872,float('nan'),True,True)#A99
	res.AppendSegment(91,-0.872+0.16,float('nan'),True,True)#S95
	res.AppendSegment(189,float('nan'),0.229,True,False)#A97
	res.AppendSegment(79,float('nan'),-0.914,True,False)#S83
	res.AppendSegment(63,float('nan'),-1.676,True,False)#S67
	res.AppendSegment(42,float('nan'),-2.159+0.076,True,False)#S46
	res.AppendSegment(209,float('nan'),-2.327+0.076,True,False)#A117
	res.AppendSegment(208,float('nan'),-2.681,True,False)#A116
	res.AppendSegment(27,float('nan'),-3.226+0.076,True,False)#S31
	res.AppendSegment(117,-0.305,float('nan'),False,False)#A25
	res.AppendSegment(13,-1.88+.076,float('nan'),False,False)#S14
        res.segments[len(res.segments)-1].AppendControlBorder(1,4,2,-1.75,float('nan'),False,False)
        res.segments[len(res.segments)-1].AppendControlBorder(0,4,2,-0.3034,float('nan'),False,False)
	res.AppendSegment(115,float('nan'),-3.073,False,True)#A23
	res.AppendSegment(30,float('nan'),-2.159,True,True)#S34
        res.segments[len(res.segments)-1].AppendControlBorder(2,4,2,float('nan'),-3.1,True,True)
	res.AppendSegment(36,float('nan'),-1.93,True,True)#S40

        return res
#path2-3
    elif index == 34: 

	res.AppendSegment(79,float('nan'),-0.914,True,False)#S83
	res.AppendSegment(63,float('nan'),-1.676,True,False)#S67
	res.AppendSegment(42,float('nan'),-2.159+0.076,True,False)#S46
	res.AppendSegment(209,float('nan'),-2.327+0.076,True,False)#A117
	res.AppendSegment(208,float('nan'),-2.681,True,False)#A116
	res.AppendSegment(27,float('nan'),-3.226+0.076,True,False)#S31
	res.AppendSegment(117,-0.305,float('nan'),False,False)#A25
	res.AppendSegment(13,-1.88+.076,float('nan'),False,False)#S14
	res.AppendSegment(115,float('nan'),-3.073,False,True)#A23
	res.AppendSegment(30,float('nan'),-2.159,True,True)#S34
	res.AppendSegment(36,float('nan'),-1.93,True,True)#S40
	res.AppendSegment(41,float('nan'),-1.372,True,True)#S45
	res.AppendSegment(73,float('nan'),-0.92,True,True)#S77
	res.AppendSegment(74,float('nan'),-0.69,True,True)#S78
	res.AppendSegment(194,-1.7,float('nan'),True,True)#A102
	res.AppendSegment(192,float('nan'),0.35-.076,True,True)#A100
	res.AppendSegment(191,-0.872,float('nan'),True,True)#A99
	res.AppendSegment(91,-0.872+0.16,float('nan'),True,True)#S95
	res.AppendSegment(189,float('nan'),0.229,True,False)#A97






        return res
#Path1(35,36,37)- it does not have control zone in it!

    elif index == 35:
	res.AppendSegment(30,float('nan'),-2.159,True,True)#S34
	res.AppendSegment(36,float('nan'),-1.93,True,True)#S40
	res.AppendSegment(41,float('nan'),-1.372,True,True)#S45
	res.AppendSegment(73,float('nan'),-0.92,True,True)#S77
	res.AppendSegment(74,float('nan'),-0.69,True,True)#S78
	res.AppendSegment(194,-1.7,float('nan'),True,True)#A102
	res.AppendSegment(192,float('nan'),0.35-.076,True,True)#A100
	res.AppendSegment(191,-0.872,float('nan'),True,True)#A99
	res.AppendSegment(91,-0.872+0.16,float('nan'),True,True)#S95
	res.AppendSegment(189,float('nan'),0.229,True,False)#A97
	res.AppendSegment(79,float('nan'),-0.914,True,False)#S83
	res.AppendSegment(63,float('nan'),-1.676,True,False)#S67
	res.AppendSegment(42,float('nan'),-2.159+0.076,True,False)#S46
	res.AppendSegment(209,float('nan'),-2.327+0.076,True,False)#A117
	res.AppendSegment(208,float('nan'),-2.681,True,False)#A116
	res.AppendSegment(27,float('nan'),-3.226+0.076,True,False)#S31
	res.AppendSegment(117,-0.305,float('nan'),False,False)#A25
	res.AppendSegment(13,-1.88+.076,float('nan'),False,False)#S14
	res.AppendSegment(115,float('nan'),-3.073,False,True)#A23

        return res

    elif index == 36:

	res.AppendSegment(36,float('nan'),-1.93,True,True)#S40
	res.AppendSegment(41,float('nan'),-1.372,True,True)#S45
	res.AppendSegment(73,float('nan'),-0.92,True,True)#S77
	res.AppendSegment(74,float('nan'),-0.69,True,True)#S78
	res.AppendSegment(194,-1.7,float('nan'),True,True)#A102
	res.AppendSegment(192,float('nan'),0.35-.076,True,True)#A100
	res.AppendSegment(191,-0.872,float('nan'),True,True)#A99
	res.AppendSegment(91,-0.872+0.16,float('nan'),True,True)#S95
	res.AppendSegment(189,float('nan'),0.229,True,False)#A97
	res.AppendSegment(79,float('nan'),-0.914,True,False)#S83
	res.AppendSegment(63,float('nan'),-1.676,True,False)#S67
	res.AppendSegment(42,float('nan'),-2.159+0.076,True,False)#S46
	res.AppendSegment(209,float('nan'),-2.327+0.076,True,False)#A117
	res.AppendSegment(208,float('nan'),-2.681,True,False)#A116
	res.AppendSegment(27,float('nan'),-3.226+0.076,True,False)#S31
	res.AppendSegment(117,-0.305,float('nan'),False,False)#A25
	res.AppendSegment(13,-1.88+.076,float('nan'),False,False)#S14
	res.AppendSegment(115,float('nan'),-3.073,False,True)#A23
	res.AppendSegment(30,float('nan'),-2.159,True,True)#S34

        return res

    elif index == 37:
	res.AppendSegment(41,float('nan'),-1.372,True,True)#S45
	res.AppendSegment(73,float('nan'),-0.92,True,True)#S77
	res.AppendSegment(74,float('nan'),-0.69,True,True)#S78
	res.AppendSegment(194,-1.7,float('nan'),True,True)#A102
	res.AppendSegment(192,float('nan'),0.35-.076,True,True)#A100
	res.AppendSegment(191,-0.872,float('nan'),True,True)#A99
	res.AppendSegment(91,-0.872+0.16,float('nan'),True,True)#S95
	res.AppendSegment(189,float('nan'),0.229,True,False)#A97
	res.AppendSegment(79,float('nan'),-0.914,True,False)#S83
	res.AppendSegment(63,float('nan'),-1.676,True,False)#S67
	res.AppendSegment(42,float('nan'),-2.159+0.076,True,False)#S46
	res.AppendSegment(209,float('nan'),-2.327+0.076,True,False)#A117
	res.AppendSegment(208,float('nan'),-2.681,True,False)#A116
	res.AppendSegment(27,float('nan'),-3.226+0.076,True,False)#S31
	res.AppendSegment(117,-0.305,float('nan'),False,False)#A25
	res.AppendSegment(13,-1.88+.076,float('nan'),False,False)#S14
	res.AppendSegment(115,float('nan'),-3.073,False,True)#A23
	res.AppendSegment(30,float('nan'),-2.159,True,True)#S34

        return res
#path3
    elif index == 38: 
	res.AppendSegment(90,-0.951+0.076,float('nan'),False,True)#S94
	res.AppendSegment(198,-2.336+0.076,0.884+0.076,False,False)#A106
	res.AppendSegment(197,-2.329-0.076,0.278,True,False)#A105
	res.AppendSegment(193,-1.617,-0.091,True,True)#A101
	res.AppendSegment(192,float('nan'),0.35-.076,True,True)#A100
	res.AppendSegment(191,-0.872,float('nan'),True,True)#A99
	res.AppendSegment(91,-0.872+0.16,float('nan'),True,True)#S95
	res.AppendSegment(87,0.713-0.076,float('nan'),True,True)#S91
	res.AppendSegment(85,1.196-0.076,float('nan'),True,True)#S89
	res.AppendSegment(184,1.685-0.076,0.347+0.076,True,False)#A92
	res.AppendSegment(179,2.065-0.076,float('nan'),True,False)#A87
	res.AppendSegment(176,2.73,0.359-0.076,True,True)#A84
	res.AppendSegment(173,2.159+0.0716,float('nan'),False,True)#A81


        return res


    elif index == 40:#Start on super long straight for merging demonstration

        res.AppendSegment(7,0.381+0.1,float('nan'),False,True) #0.46
        res.AppendSegment(8,float('nan'),-0.8382+0.02,True,True)
        res.AppendSegment(9,float('nan'),0.3010,True,True)

        res.AppendSegment(10,0.6369,float('nan'),True,True)

        res.AppendSegment(4,1.1963,float('nan'),True,True)

        res.AppendSegment(5,float('nan'),0.5,True,False)#start of the arc in the western roundabout 
        res.AppendSegment(14,1.997,float('nan'),True,True)
	res.AppendSegment(11,float('nan'),-0.2032,True,False) #added this arc back in
        res.AppendSegment(12,float('nan'),-0.8382+0.02,True,False) #-0.854
        res.AppendSegment(13,2.03,float('nan'),False,True) #2.00304

        return res
    elif index == 41: 
	
	res.AppendSegment(63,float('nan'),-1.676,True,False)#S67
	res.AppendSegment(42,float('nan'),-2.159+0.076,True,False)#S46
	res.AppendSegment(209,float('nan'),-2.327+0.076,True,False)#A117
	res.AppendSegment(208,float('nan'),-2.681,True,False)#A116
	res.AppendSegment(27,float('nan'),-3.226+0.076,True,False)#S31
	res.AppendSegment(117,-0.305,float('nan'),False,False)#A25
	res.AppendSegment(13,-1.88+.076,float('nan'),False,False)#S14
	res.AppendSegment(115,float('nan'),-3.073,False,True)#A23
	res.AppendSegment(30,float('nan'),-2.159,True,True)#S34
	res.AppendSegment(36,float('nan'),-1.93,True,True)#S40
	res.AppendSegment(41,float('nan'),-1.372,True,True)#S45
	res.AppendSegment(73,float('nan'),-0.92,True,True)#S77
	res.AppendSegment(74,float('nan'),-0.69,True,True)#S78
	res.AppendSegment(194,-1.7,float('nan'),True,True)#A102
	res.AppendSegment(192,float('nan'),0.35,True,True)#A100
	res.AppendSegment(191,-0.872,float('nan'),True,True)#A99
	res.AppendSegment(91,-0.872+0.16,float('nan'),True,True)#S95
	res.AppendSegment(189,float('nan'),0.229,True,False)#A97
        res.AppendSegment(79,float('nan'),-0.914,True,False)#S83

        return res
    	#dual dynamic roundabouts (control zones need to be the same as the roundabouts) (upper start)
	#implementing float("nan") into the y values of all the indexes 
	#steps to fix MergeBuilder: make control zones (difference between 0 and 1 appendcontrolborder statments) the same for all indexes
	# Difference between x values of or and 1 appendcontroborder statments is L
	#Make merge zones (difference between 1 and 2 appendcontrolborder statments) the same and SMALLER (0.4-0.6) for all indexes 
	# Difference between x values of 1 and 2 appendcontrolborder statments is S
	# How to calculate delta value? Delta value is the saftey length so try 0.5
    elif index == 39: 

	res.AppendSegment(87,0.713-0.076,float('nan'),True,True)#S91
	res.AppendSegment(85,1.196-0.076,float('nan'),True,True)#S89
	res.AppendSegment(184,1.685-0.076,0.347+0.076,True,False)#A92
	res.AppendSegment(179,2.065-0.076,float('nan'),True,False)#A87
	res.AppendSegment(176,2.73,0.359-0.076,True,True)#A84
	res.AppendSegment(173,2.159+0.0716,float('nan'),False,True)#A81
	res.AppendSegment(90,-0.951+0.076,float('nan'),False,True)#S94
        #res.segments[len(res.segments)-1].AppendControlBorder(2,4,0,0,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(0,4,0,-1.3,float('nan'),True,True)
	res.AppendSegment(198,-2.336+0.076,0.884+0.076,False,False)#A106
        res.segments[len(res.segments)-1].AppendControlBorder(1,4,0,-1.6,0.72,True,True)
	res.AppendSegment(197,-2.329-0.076,0.278,True,False)#A105
        res.segments[len(res.segments)-1].AppendControlBorder(2,4,0,-2.5,0.83,True,True)
	res.AppendSegment(193,-1.617,-0.091,True,True)#A101
	res.AppendSegment(192,float('nan'),0.35-0.076,True,True)#A100*
	res.AppendSegment(191,-0.872-.076,float('nan'),True,True)#A99*
	res.AppendSegment(91,-0.872+0.16,float('nan'),True,True)#S95


        return res

    elif index == 42: 

	res.AppendSegment(85,1.196-0.076,float('nan'),True,True)#S89
	res.AppendSegment(184,1.685-0.076,0.347+0.076,True,False)#A92
	res.AppendSegment(179,2.065-0.076,float('nan'),True,False)#A87
	res.AppendSegment(176,2.73,0.359-0.076,True,True)#A84
	res.AppendSegment(173,2.159+0.0716,float('nan'),False,True)#A81
	res.AppendSegment(90,-0.951+0.076,float('nan'),False,True)#S94
	res.AppendSegment(198,-2.336+0.076,0.884+0.076,False,False)#A106
	res.AppendSegment(197,-2.329-0.076,0.278,True,False)#A105
	res.AppendSegment(193,-1.617,-0.091,True,True)#A101
	res.AppendSegment(192,float('nan'),0.35-0.076,True,True)#A100*
	res.AppendSegment(191,-0.872-.076,float('nan'),True,True)#A99*
	res.AppendSegment(91,-0.872+0.16,float('nan'),True,True)#S95
	res.AppendSegment(87,0.713-0.076,float('nan'),True,True)#S91

        return res

    elif index == 43: 


	res.AppendSegment(184,1.685-0.076,0.347+0.076,True,False)#A92
	res.AppendSegment(179,2.065-0.076,float('nan'),True,False)#A87
	res.AppendSegment(176,2.73,0.359-0.076,True,True)#A84
	res.AppendSegment(173,2.159+0.0716,float('nan'),False,True)#A81
	res.AppendSegment(90,-0.951+0.076,float('nan'),False,True)#S94
	res.AppendSegment(198,-2.336+0.076,0.884+0.076,False,False)#A106
	res.AppendSegment(197,-2.329-0.076,0.278,True,False)#A105
	res.AppendSegment(193,-1.617,-0.091,True,True)#A101
	res.AppendSegment(192,float('nan'),0.35-0.076,True,True)#A100*
	res.AppendSegment(191,-0.872-.076,float('nan'),True,True)#A99*
	res.AppendSegment(91,-0.872+0.16,float('nan'),True,True)#S95

	res.AppendSegment(87,0.713-0.076,float('nan'),True,True)#S91
	res.AppendSegment(85,1.196-0.076,float('nan'),True,True)#S89

        return res
    
    # dual dynamic roundabouts lower start (control zones need to be the same as the roundabouts) 
    elif index == 44:
        res.AppendSegment(90,-0.951+0.076,float('nan'),False,True)#S94
        #res.segments[len(res.segments)-1].AppendControlBorder(2,4,0,0,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(0,4,0,-1.3,float('nan'),True,True)
        res.AppendSegment(198,-2.336+0.076,0.884+0.076,False,False)#A106
        res.segments[len(res.segments)-1].AppendControlBorder(1,4,0,-1.6,0.72,True,True)
        res.AppendSegment(197,-2.329-0.076,0.278,True,False)#A105
        res.segments[len(res.segments)-1].AppendControlBorder(2,4,0,-2.5,0.83,True,True) # potenial issue is different ending of merging zones and float nan of roundabout
        res.AppendSegment(193,-1.617,-0.091,True,True)#A101
        res.AppendSegment(192,float('nan'),0.35-0.076,True,True)#A100*
        res.AppendSegment(191,-0.872-.076,float('nan'),True,True)#A99*
        res.AppendSegment(91,-0.872+0.16,float('nan'),True,True)#S95
        res.AppendSegment(87,0.713-0.076,float('nan'),True,True)#S91
        res.AppendSegment(85,1.196-0.076,float('nan'),True,True)#S89
        res.AppendSegment(184,1.685-0.076,0.347+0.076,True,False)#A92
        res.AppendSegment(179,2.065-0.076,float('nan'),True,False)#A87
        res.AppendSegment(176,2.73,0.359-0.076,True,True)#A84
        res.AppendSegment(173,2.159+0.0716,float('nan'),False,True)#A81
        return res
    
    #right western roundabout (control zones need to be the same as the roundabouts) 
    elif index == 45:
        
        res.AppendSegment(197,-2.329-0.076,0.278,True,False)#A105
        res.AppendSegment(193,-1.617,-0.091,True,True)#A101
        res.AppendSegment(192,float('nan'),0.35-0.076,True,True)#A100*
        res.segments[len(res.segments)-1].AppendControlBorder(0,4,1,-1.45,0.51,True,True)
        res.AppendSegment(200,-2.07,float('nan'),False,False)#-2.336,0.884 (most likely)
        res.segments[len(res.segments)-1].AppendControlBorder(1,4,1,-1.75,0.86,True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(2,4,1,-2.5,0.63,True,True)
        return res
    
    #right northern roundabout (control zones need to be the same as the roundabouts) 
    elif index == 46:
        res.AppendSegment(193,-1.617,-0.091,True,True)#A101
        res.AppendSegment(192,float('nan'),0.35-0.076,True,True)#A100*
        res.AppendSegment(200,-2.07,float('nan'),False,False)#A108
        res.segments[len(res.segments)-1].AppendControlBorder(1,4,1,-1.45,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(0,4,1,-1.74,float('nan'),True,True)
        res.AppendSegment(197,-2.329-0.076,0.278,True,False)#A105
        res.segments[len(res.segments)-1].AppendControlBorder(2,4,1,-2.5,0.63,True,True)
        return res
    #right northwestern roundabout (control zones need to be the same as the roundabouts) 
    elif index == 47:
        res.AppendSegment(192,float('nan'),0.35-0.076,True,True)#A100*
        
        res.AppendSegment(200,-2.07,float('nan'),False,False)#A108
        res.segments[len(res.segments)-1].AppendControlBorder(1,4,1,-1.45,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(0,4,1,-1.74,float('nan'),True,True)
        
        res.AppendSegment(197,-2.329-0.076,0.278,True,False)#A105
        res.segments[len(res.segments)-1].AppendControlBorder(2,4,1,-2.5,0.63,True,True)
        res.AppendSegment(193,-1.617,-0.091,True,True)#A101
        return res
    #right southern roundabout (control zones need to be the same as the roundabouts) 
    elif index == 48:
        res.AppendSegment(193,-1.617,-0.091,True,True)#A101
        res.AppendSegment(192,float('nan'),0.35-0.076,True,True)#A100*
        res.AppendSegment(200,-2.07,float('nan'),False,False)#A108
        res.segments[len(res.segments)-1].AppendControlBorder(1,4,1,-1.45,float('nan'),True,True)
        res.segments[len(res.segments)-1].AppendControlBorder(0,4,1,-1.74,float('nan'),True,True)
        res.AppendSegment(197,-2.329-0.076,0.278,True,False)#A105
        res.segments[len(res.segments)-1].AppendControlBorder(2,4,1,-2.5,0.63,True,True)
        return res
    
    
    #left southeastern roundabout (needs control zones)
    elif index == 49:
        res.AppendSegment(176,2.238,float('nan'),True,True) #a89
        res.AppendSegment(179,2.065-0.076,float('nan'),True,False)#A87
        res.AppendSegment(176,2.73,0.359-0.076,True,True)#A84
        res.AppendSegment(173,2.159+0.0716,float('nan'),False,True)#A81
        return res
    #left northeastern roundabout (needs control zones)
    elif index == 50:
        res.AppendSegment(179,2.065-0.076,float('nan'),True,False)#A87
        res.AppendSegment(176,2.73,0.359-0.076,True,True)#A84
        res.AppendSegment(173,2.159+0.0716,float('nan'),False,True)#A81
        res.AppendSegment(176,2.238,float('nan'),True,True) #a89
        return res
    #left northwestern roundabout (needs control zones)
    elif index == 51:
        res.AppendSegment(176,2.73,0.359-0.076,True,True)#A84
        res.AppendSegment(173,2.159+0.0716,float('nan'),False,True)#A81
        res.AppendSegment(176,2.238,float('nan'),True,True) #a89
        res.AppendSegment(179,2.065-0.076,float('nan'),True,False)#A87
        return res
    #left eastern roundabout (needs control zones)
    elif index == 52:
        res.AppendSegment(173,2.159+0.0716,float('nan'),False,True)#A81
        res.AppendSegment(176,2.238,float('nan'),True,True) #a89
        res.AppendSegment(179,2.065-0.076,float('nan'),True,False)#A87
        res.AppendSegment(176,2.73,0.359-0.076,True,True)#A84
        return res


    










#################################################################################################################

#   if index == 29: #Test new version of mapbuilder
	#res.AppendSegment(4,1.1263,float('nan'),True,True) #comment by Nikhil for testing
#	res.AppendSegment(0,-0.9506,float('nan'),False,True)

#        return res
    else:
        print "No default path id found"