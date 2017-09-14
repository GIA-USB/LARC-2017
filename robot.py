from Dynamics import DifferentialDrive
from Pose import Pose
from rotaryEncoder import Encoder
import pigpio
from math import pi

class ticks:

    def __init__(self, l, r):
        self.left = l
        self.right = r

class Robot:

    def __init__(self, wheelRadius, wheelBaseLength, ticksPerRev, minRPM, maxRPM, 
                 minVel, maxVel):
        self.wheelRadius = wheelRadius # En metros.
        self.wheelBaseLength = wheelBaseLength # En metros.
        self.ticksPerRev = ticksPerRev #Entero
        self.minRPM = minRPM
        self.maxRPM = maxRPM
        self.minVel = minRPM * 2 * pi / 60 # En rad/s
        self.maxVel = maxRPM * 2 * pi / 60 # En rad/s
        self.encoders = []
        self.irArray = []
        self.dynamics = DifferentialDrive(self.wheelRadius, self.wheelBaseLength);
        self.prevTicks = ticks(0,0)
        self.rightWheelSpeed = 0
        self.leftWheelSpeed = 0
        self.stateEstimate = Pose()
        self.pi = pigpio.pi()

    def setEncoders(self, pins):
        nPins = len(pins)
        i = 0
        while (i < nPins):
            self.encoders.append(Encoder(self.pi, pins[i], pins[i+1]))
            i += 2
            
    def getUSDistances(self):
        ir_array_values = self.ir_array.get_range()
        return ir_distances

    '''
    def updateState(self, pose, dt):
        R = self.wheelRadius
        velR = self.rightWheelSpeed     # mm/s
        velL = self.leftWheelSpeed      # mm/s
        
        pose = self.dynamics.apply_dynamics(pose, dt, velR, velL)
        self.update_pose(pose)
        
        for k=1:length(self.ir_array)
            self.ir_array(k).update_pose(pose)
        end
        
        # Update wheel encoders
        
        velR = self.right_wheel_speed # mm/s
        velL = self.left_wheel_speed  # mm/s
        
        self.encoders(1).update_ticks(velR, dt)
        self.encoders(2).update_ticks(velL, dt)
        return pose
    '''
        
    def setWheelSpeeds(self, velR, velL):
        [velR, velL] = self.limitSpeeds(velR, velL)
        self.rightWheelSpeed = velR
        self.leftWheelSpeed = velL
    
    def limitSpeeds(self, velR, velL):
        # Actuator hardware limits            
        velR = max(min(velR, self.maxVel), -self.maxVel);
        velL = max(min(velL, self.maxVel), -self.maxVel);
        
        velR = velR*(abs(velR) >= self.minVel);
        velL = velL*(abs(velL) >= self.minVel);
        
        return [velR, velL]