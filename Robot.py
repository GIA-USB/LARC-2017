from Dynamics import DifferentialDrive
from Pose import Pose
from DCMotor import Motor
from rotaryEncoder import Encoder
from IMU import IMU
from Ultrasonic import Ultrasonic
from math import pi, radians
from numpy import sign
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
		self.minVel = 0 #0.150006681210744#0#3.2498914648863666#0.0394496029208 # En rad/s
		self.maxVel = 1.0262706579357654#5.150818988820947#0.0433583709166 # En rad/s
		self.beta = [0.0050072227207064515, -0.25057113358444087]       
		self.prevTicks = ticks(0,0)
		self.rightWheelSpeed = 0
		self.leftWheelSpeed = 0
		self.stateEstimate = Pose(x = 0, y = 0, theta = 0, yawO = 0)
		self.dynamics = DifferentialDrive(self.wheelRadius, self.wheelBaseLength);
		self.pi = pigpio.pi()
		self.rightMotor = None
		self.leftMotor = None
		self.leftEnconder = None
		self.rightEncoder = None
		self.imu = None
		self.ultrasonics = []
        
	def setMotors(self, M1IN1, M1IN2, M2IN1, M2IN2):
		self.leftMotor = Motor(self.pi, M1IN1, M1IN2)
		self.rightMotor = Motor(self.pi, M2IN1, M2IN2)
		
	def setEncoders(self, E1A, E1B, E2A, E2B):
		self.rightEncoder = Encoder(self.pi, E1A, E1B)
		self.leftEncoder = Encoder(self.pi, E2A, E2B)
		
	def setIMU(self):
		self.imu = IMU()
		
	def getYaw(self):
		value = ""
		req = open('requestPipe', "w")
		req.write('ok')
		req.close()
		while (len(value) == 0):
			resp = open('responsePipe','r') 
			value = resp.read()
			resp.close()
		value = value.split()
		value = float(value[0]) * sign(float(value[1]))
		print("Angulo Original: " + str(value))
		resp.close()
		resp = open('responsePipe','w')
		resp.write("")
		resp.close()
		if (value > 0):
			value = 180 - value
		else:
			value = 360 + value
		print("Angulo transformado: " + str(value))
		return value
		
	def setUltrasonics(self,TRIGGER,ECHO1,ECHO2,ECHO3,ECHO4,ECHO5,ECHO6):
		self.ultrasonics.append(Ultrasonic(self.pi,None,ECHO1))
		self.ultrasonics.append(Ultrasonic(self.pi,None,ECHO2))
		self.ultrasonics.append(Ultrasonic(self.pi,None,ECHO3))
		self.ultrasonics.append(Ultrasonic(self.pi,None,ECHO4))
		self.ultrasonics.append(Ultrasonic(self.pi,None,ECHO5))
		self.ultrasonics.append(Ultrasonic(self.pi,TRIGGER,ECHO6))
		
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
		
		rightPWM = sign(self.rightWheelSpeed) * (abs(self.rightWheelSpeed) - self.beta[1]) / self.beta[0]
		leftPWM = sign(self.leftWheelSpeed) * (abs(self.leftWheelSpeed) - self.beta[1]) / self.beta[0]
		print("RightPWM: " + str(rightPWM))
		print("LeftPWM: " + str(leftPWM))
		#rightPWM = max(min(round(rightPWM), 100), -100)
		#leftPWM = max(min(round(leftPWM), 100), -100)
		'''
		if (rightPWM > 255):
			rightPWM = 255
		if (leftPWM > 255):
			leftPWM = 255
		print("PWM1: " + str(rightPWM))
		print("PWM2: " + str(leftPWM)) 
		'''
		# Debe considerarse el caso de PWM negativo.
		'''
		if(rightPWM == 175):
			rightPWM = 0
		if(leftPWM == 175):
			leftPWM = 0
		'''
		if (rightPWM < 0):
			if (leftPWM < 0):
				self.rightMotor.setMotorPWM(abs(rightPWM),0)
				self.leftMotor.setMotorPWM(0,abs(leftPWM))
			else:
				self.rightMotor.setMotorPWM(abs(rightPWM),0)
				self.leftMotor.setMotorPWM(leftPWM,0)
		else:
			if (leftPWM < 0):
				self.rightMotor.setMotorPWM(0,rightPWM)
				self.leftMotor.setMotorPWM(0,abs(leftPWM))
			else:
				self.rightMotor.setMotorPWM(0,rightPWM)
				self.leftMotor.setMotorPWM(leftPWM,0)

	def limitSpeeds(self, velR, velL):
		# Actuator hardware limits
		velR = max(min(velR, self.maxVel), -self.maxVel);
		velL = max(min(velL, self.maxVel), -self.maxVel);
		velR = velR*(abs(velR) >= self.minVel);
		velL = velL*(abs(velL) >= self.minVel);
		
		return [velR, velL]
