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

		#inputs = struct('x_g', 0, 'y_g', 0, 'v', 0);
        #outputs = struct('v', 0, 'w', 0);

	def execute(self, robot, stateEstimate, inputs, dt):
		# Compute the placement of the sensors.
		if (not(self.calibrated)):
			self.setSensorGeometry(robot)
            
		# Unpack state estimate.
		x, y, theta = stateEstimate.unpack()
            
		# Poll the current IR sensor values.
		irDistances = robot.getIrDistances()

		# Interpret the IR sensor measurements geometrically.
		irDistancesWF = self.applySensorGeometry(irDistances, stateEstimate)

		# Compute the heading vector for obstacle avoidance.
		sensorGains = np.array([1, 1, 0.5, 1, 1])
		u_i = np.dot((irDistancesWF - np.tile(np.array([[x],[y]]),1,5)), np.diag(sensorGains))
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

	def applySensorGeometry(self, irDistances, stateEstimate):
		nSensors = len(irDistances)

    	# Apply the transformation to robot frame.
		irDistancesRF = np.zeros([3,5]);
		for i in range(nSensors):
			x_s = self.sensorPlacement[1,i]
			y_s = self.sensorPlacement[2,i]
			theta_s = self.sensorPlacement[3,i]
			R = self.getTransformationMatrix(x_s, y_s, theta_s)
			irDistancesRF[:,i] = np.dot(R, np.array([ir_distances[i], 0, 1]))
           
		# Apply the transformation to world frame.
		[x,y,theta] = stateEstimate.unpack()
		R = self.get_transformation_matrix(x,y,theta)
		irDistancesWF = np.dot(R, irDistancesRF)
		irDistancesWF = irDistancesWF[0:2,:]

		return irDistancesWF

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