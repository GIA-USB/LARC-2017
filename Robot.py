from Dynamics import DifferentialDrive
from Pose import Pose
from DCMotor import Motor
from rotaryEncoder import Encoder
from math import pi
import pigpio


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
		#self.minVel = minRPM * 2 * pi / 60 # En rad/s
		#self.maxVel = maxRPM * 2 * pi / 60 # En rad/s
		self.minVel = 0.0394496029208 # En rad/s
		self.maxVel = 0.0433583709166 # En rad/s
		self.beta = [4.88595999478e-05, -0.0519088009075]       
		self.prevTicks = ticks(0,0)
		self.rightWheelSpeed = 0
		self.leftWheelSpeed = 0
		self.stateEstimate = Pose()
		self.dynamics = DifferentialDrive(self.wheelRadius, self.wheelBaseLength);
		self.pi = pigpio.pi()
		self.rightMotor = None
		self.leftMotor = None
		self.leftEnconder = None
		self.rightEncoder = None
		self.irArray = []
        
	def setMotors(self, M1IN1, M1IN2, M2IN1, M2IN2):
		self.rightMotor = Motor(self.pi, M1IN1, M1IN2)
		self.leftMotor = Motor(self.pi, M2IN1, M2IN2)
		
	def setEncoders(self, E1A, E1B, E2A, E2B):
		self.rightEncoder = Encoder(self.pi, E1A, E1B)
		self.leftEncoder = Encoder(self.pi, E2A, E2B)
		
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
    
	def setMotorsSpeeds(self, velR, velL):
		#[velR, velL] = self.limitSpeeds(velR, velL)
		self.rightWheelSpeed = velR
		self.leftWheelSpeed = velL
		
		#right_rps = obj.right_wheel_speed/(2*pi);
		#left_rps = obj.left_wheel_speed/(2*pi);
		
		rightPWM = sign(rightWheelSpeed) * (abs(rightWheelSpeed) - beta[1]) / beta[0]
		leftPWM = sign(leftWheelSpeed) * (abs(leftWheelSpeed) - beta[1]) / beta[0]
		
		rightPWM = max(min(round(rightPWM), 100), -100)
		leftPWM = max(min(round(leftPWM), 100), -100)
		
		# Debe considerarse el caso de PWM negativo.
		rightMotor.setMotorPWM(rightPWM,0)
		leftMotor.setMotorPWM(leftPWM,0)

	def limitSpeeds(self, velR, velL):
		# Actuator hardware limits
		velR = max(min(velR, self.maxVel), -self.maxVel);
		velL = max(min(velL, self.maxVel), -self.maxVel);
		velR = velR*(abs(velR) >= self.minVel);
		velL = velL*(abs(velL) >= self.minVel);
		
		return [velR, velL]
