from math import sin,cos,atan2,pi

class Pose:
    
    def __init__(self, x = 0, y = 0, theta = 0):
        self.x = x
        self.y = y
        self.theta = theta
    
    def setPose(self, pose):
        self.x = pose[0]
        self.y = pose[1]
        self.theta = pose[2]
    
    def unpack(self):
        return [self.x, self.y, self.theta]
    
    def getTransformationMatrix(self):
        T = [[cos(self.theta), -sin(self.theta), self.x],
             [sin(self.theta), cos(self.theta), self.y],
             [0, 0, 1]]
        return T
    
    def deg2Rad(self, deg):
        return deg * pi / 180
        
    def rad2Deg(self, rad):
        return rad * 180 / pi