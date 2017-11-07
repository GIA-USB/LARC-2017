from math import sin,cos,atan2
import numpy as np

class avoidObstacles:
	def __init__(self, kp, ki, kd):
		# Gains.
		self.Kp = kp
		self.Ki = ki
		self.Kd = kd
		
        # Memory banks.
		self.errorAcum = 0
		self.errorPrev = 0

		calibrated = False
		sensorPlacement = np.zeros([3,5])
		minDistance = -1
		#inputs = struct('x_g', 0, 'y_g', 0, 'v', 0);
        #outputs = struct('v', 0, 'w', 0);

	def execute(self, robot, stateEstimate, inputs, dt):
		# Compute the placement of the sensors.
		'''
		if (not(self.calibrated)):
			self.setSensorGeometry(robot)
        '''

		# Unpack state estimate.
		x, y, theta = stateEstimate.unpack()
            
		# Poll the current IR sensor values.
		usDistances = np.array([0,0,0])
		for i in range (0,10):
			usDistances = usDistances + np.array(robot.getUSDistances())
		usDistances /= 10
		self.minDistance = min(usDistances)

		# Interpret the IR sensor measurements geometrically.
		usDistancesWF = self.applySensorGeometry(usDistances, robot.usLocation, stateEstimate)

		# Compute the heading vector for obstacle avoidance.
		sensorGains = np.array([1, 0.5, 1])
		u_i = np.dot((usDistancesWF - np.tile(np.array([[x],[y]]),1,3)), np.diag(sensorGains))
		u_ao = np.sum(u_i,1);
            
		# Compute the heading and error for the PID controller.
		theta_ao = atan2(u_ao[2],u_ao[1])
		error = theta_ao - theta
		error = atan2(sin(error),cos(error))
		eIntegral = self.errorAcum + error * dt
		eDerivate = (error - self.errorPrev) / dt

		# PID control on w.
		v = inputs.v
		w = self.Kp * error + self.Ki * eIntegral + self.Kd * eDerivate
            
		# Save errors for next time step.
		self.errorAcum = eIntegral
		self.errorPrev = error

		return v,w

	def applySensorGeometry(self, usDistances, usLocation, stateEstimate):
		nSensors = len(usDistances)

    	# Apply the transformation to robot frame.
		usDistancesRF = np.zeros([3,nSensors]);
		for i in range(nSensors):
			x_s = usLocation[i][0]
			y_s = usLocation[i][1]
			theta_s = usLocation[i][2]
			R = self.getTransformationMatrix(x_s, y_s, theta_s)
			usDistancesRF[:,i] = np.dot(R, np.array([usDistances[i], 0, 1]))
           
		# Apply the transformation to world frame.
		[x,y,theta] = stateEstimate.unpack()
		R = self.getTransformationMatrix(x,y,theta)
		usDistancesWF = np.dot(R, usDistancesRF)
		usDistancesWF = usDistancesWF[0:2,:]

		return usDistancesWF

	def setSensorGeometry(self, robot):
		for i in range(5):
			x, y, theta = robot.ir_array(i).location.unpack()
			self.sensorPlacement[:,i] = np.array([x, y, theta])
		self.calibrated = True

	def getTransformationMatrix(self, x, y, theta):
		return np.array([[cos(theta), -sin(theta), x], [sin(theta), cos(theta), y], [0, 0, 1]]) 

	# Reset accumulated and previous error.
	def reset():
		self.errorAcum = 0;
		self.errorPrev = 0;

	def checkTransitions(self, robot, stateEstimate, inputs):
		if (self.minDistance > 0.1):
			return 1
		else:
			return 3