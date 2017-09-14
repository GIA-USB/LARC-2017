class DifferentialDrive():
    
    def __init__(self, wheelRadius, wheelBaseLength):
        self.wheelRadius = wheelRadius
        self.wheelBaseLength = wheelBaseLength

    def uniToDiff(self, v, w):
        R = self.wheelRadius
        L = self.wheelBaseLength
        velR = (2 * v + w * L) / (2 * R)
        velL = (2 * v - w * L) / (2 * R)
        
        return [velR, velL] 
        
    def diffToUni(self, r, l):
        R = self.wheelRadius
        L = self.wheelBaseLength
        v = R / 2 * (r + l)
        w = R / L * (r - l)
        
        return [v, w]
