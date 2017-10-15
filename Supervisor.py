from math import sin,cos,atan2,pi, sqrt
from Stop import *
from GoToAngle import *
from GoToGoal import *
from numpy import sign
import sys

class Supervisor():

	def __init__(self, robot, thetaD = pi/4, v = 0.15, goal = [-1, 0], dStop = 0.05,
	     dAtObs = 0.25, dUnsafe = 0.10, d_fw = 0.001, 
	     fwDirection = 'left', isBlending = True):
		# Supervisor constructor.

		self.robot = robot
		# Initialize the controllers
		self.controllers = []
		self.controllers.append(Stop())
		self.controllers.append(GoToAngle())
		self.controllers.append(GoToGoal(0.3,0.5,0.3))
		#self.controllers.append(goToGoal(1,1.5,1))
		#self.controllers.append(AvoidObstacles())
		#self.controllers.append(AOandGTG())
		#self.controllers.append(FollowWall())

		# set the initial controller
		self.currentController = self.controllers[1]
		self.thetaD = thetaD
		self.v = v
		self.goal = goal
		self.dStop = dStop
		self.dAtObs = dAtObs                
		self.dUnsafe = dUnsafe
		self.d_fw = d_fw
		self.fwDirection = fwDirection
		self.isBlending = isBlending

	def execute(self, dt):
		# EXECUTE Selects and executes the current controller.
		#   execute(obj, robot) will select a controller from the list of
		#   available controllers and execute it.
		#
		#   See also controller/execute

		self.updateOdometry()
		inputs = self.controllers[0].inputs 
		inputs.v = self.v
		inputs.xGoal = self.goal[0]
		inputs.yGoal = self.goal[1]
		#inputs.direction = self.fwDirection
		#inputs.d_fw = self.d_fw
		outputs = self.currentController.execute(self.robot, self.robot.stateEstimate, inputs, dt)
		print("Ov =" + str(outputs.v) + " Ow = " + str(outputs.w))
		#[velR, velL] = self.robot.dynamics.uniToDiff(outputs.v,outputs.w)
		[velR, velL] = self.ensureW(outputs.v, outputs.w)
		if (velR < 0 or velL < 0):
			print("GRITAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
		print("VelR = " + str(velR) + " VelL = " + str(velL))
		current = self.currentController.checkTransitions(self.robot, self.robot.stateEstimate, inputs)
		self.currentController = self.controllers[current]
		self.robot.setMotorsSpeeds(velR,velL)
		
	def updateOdometry(self):
		'''
		UPDATE_ODOMETRY Approximates the location of the robot.
		obj = obj.update_odometry(robot) should be called from the
		execute function every iteration. The location of the robot is
		updated based on the difference to the previous wheel encoder
		ticks. This is only an approximation.

		state_estimate is updated with the new location and the
		measured wheel encoder tick counts are stored in prev_ticks.
		'''

		# Get wheel encoder ticks from the robot.
		# ESTO DEBE ADAPTARSE A NUESTROS ENCODERS.
		leftTicks = self.robot.leftEncoder.getTicks()
		rightTicks = -self.robot.rightEncoder.getTicks()
		print("Right: " + str(rightTicks) + " Left: " + str(leftTicks))
		# RecalL the previous wheel encoder ticks.
		prevRightTicks = self.robot.prevTicks.right
		prevLeftTicks = self.robot.prevTicks.left

		# Previous estimate.
		[x, y, theta] = self.robot.stateEstimate.unpack()
		print("PrevRight: " + str(prevRightTicks) + " PrevLeft: " + str(prevLeftTicks))
		# Compute odometry here
		R = self.robot.wheelRadius
		L = self.robot.wheelBaseLength
		mPerTick = (2 * pi * R) / self.robot.ticksPerRev
		print("Right resta: " + str(rightTicks - prevRightTicks))
		print("Left resta: " + str(leftTicks - prevLeftTicks))
		dRight = (rightTicks - prevRightTicks) * mPerTick
		dLeft = (leftTicks - prevLeftTicks) * mPerTick
		dCenter = (dRight + dLeft) / 2

		xGoal = self.goal[0]
		yGoal = self.goal[1]
		u_x = xGoal - x;     

		# Distance between goal and robot in y-direction.
		u_y = yGoal - y;
        # Angle from robot to goal.
		thetaGoal = atan2(u_y,u_x)
		#print("u_x: " + str(u_x) + " u_y: " + str(u_y) + " Theta: " + str(thetaGoal))
		# Calculate the heading error.
        # Error between the goal angle and robot's angle.
		error = thetaGoal - theta

		xNew = x + dCenter * cos(theta)
		yNew = y + dCenter * sin(theta)
		'''
		if(abs(error) > pi):
			thetaNew = theta + ((dRight - dLeft) / L)
		elif(abs(error) <= pi):
			thetaNew = theta + (-1*abs(((dRight - dLeft) / L)))
		'''
		print("Theta antes: " + str(theta))
		print("Lo que le voy a sumar o restar: " + str(((dRight - dLeft) / L)))
		thetaNew = theta + ((dRight - dLeft) / L)
		print("Thetanew: " + str(thetaNew))
		# fprintf('Estimated pose (x,y,theta): (%0.3g,%0.3g,%0.3g)\n', x_new, y_new, theta_new);

		# Save the wheel encoder ticks for the next estimate
		# ARREGLAR
		self.robot.prevTicks.right = rightTicks
		self.robot.prevTicks.left = leftTicks

		# Update your estimate of (x,y,theta)
		self.robot.stateEstimate.setPose([xNew, yNew, atan2(sin(thetaNew), cos(thetaNew))])

	def ensureW(self, v, w):

		# This function ensures that w is respected as best as possible
		# by scaling v.

		R = self.robot.wheelRadius
		L = self.robot.wheelBaseLength
		velMax = self.robot.maxVel
		velMin = self.robot.minVel

		if (abs(v) > 0):
			print("W: " + str(w))
			# 1. Limit v,w to be possible in the range [vel_min, vel_max]
			# (avoid stalling or exceeding motor limits)
			vLim = max(min(abs(v), (R / 2) * (2 * velMax)), (R / 2) * (2 * velMin))
			wLim = max(min(abs(w), (R / L) * (velMax - velMin)), 0) 
			print("vlim= " + str(vLim) + " wLim= " + str(wLim))
			#2. Compute the desired curvature of the robot's motion

			[velRDiff, velLDiff] = self.robot.dynamics.uniToDiff(vLim, wLim)
			print("velLDiff: " + str(velLDiff) + " velRDiff: " + str(velRDiff))
			# 3. Find the max and min velR/velL
			velRLMax = max(velRDiff, velLDiff)
			velRLMin = min(velRDiff, velLDiff)

			# 4. Shift velR and velL if they exceed max/min vel
			if (velRLMax > velMax):
			    velR = velRDiff - (velRLMax - velMax)
			    velL = velLDiff - (velRLMax - velMax)
			elif (velRLMin < velMin):
			    velR = velRDiff + (velMin - velRLMin)
			    velL = velLDiff + (velMin - velRLMin)
			else:
			    velR = velRDiff
			    velL = velLDiff

			# 5. Fix signs (Always either both positive or negative)
			[vShift, wShift] = self.robot.dynamics.diffToUni(velR, velL)
			v = sign(v) * vShift
			w = sign(w) * wShift
			print("v: " + str(v) + "w: " + str(w))
		else:
			# Robot is stationary, so we can either not rotate, or
			# rotate with some minimum/maximum angular velocity
			wMin = R / L * (2 * velMin)
			wMax = R / L * (2 * velMax)
			if (abs(w) > wMin):
			    w = sign(w) * max(min(abs(w), wMax), wMin)
			else:
			    w = 0

		return self.robot.dynamics.uniToDiff(v, w)
