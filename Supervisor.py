from math import sin,cos,atan2,pi, sqrt
from goToGoal import *
from numpy import sign
import sys

class Supervisor():

	def __init__(self, robot, thetaD = pi/4, v = 0.15, goal = [2, 0], dStop = 0.05,
	     dAtObs = 0.25, dUnsafe = 0.10, d_fw = 0.001, 
	     fwDirection = 'left', isBlending = True):
		# Supervisor constructor.

		self.robot = robot
		# Initialize the controllers
		self.controllers = []
		#self.controllers.append(Stop())
		#self.controllers.append(GoToAngle())
		self.controllers.append(goToGoal(1,1.5,1))
		#self.controllers.append(AvoidObstacles())
		#self.controllers.append(AOandGTG())
		#self.controllers.append(FollowWall())

		# set the initial controller
		self.currentController = self.controllers[0]

		'''
		obj.current_state = 6;

		% generate the set of states
		for i = 1:length(obj.controllers)
		obj.states{i} = struct('state', obj.controllers{i}.type, ...
		                       'controller', obj.controllers{i});
		end

		% define the set of eventsd
		obj.eventsd{1} = struct('event', 'at_obstacle', ...
		                   'callback', @at_obstacle);

		obj.eventsd{2} = struct('event', 'at_goal', ...
		                   'callback', @at_goal);

		obj.eventsd{3} = struct('event', 'obstacle_cleared', ...
		                    'callback', @obstacle_cleared);
		                
		obj.eventsd{4} = struct('event', 'unsafe', ...
		                    'callback', @unsafe);
		               
		obj.prev_ticks = struct('left', 0, 'right', 0);
		'''

		self.thetaD = thetaD
		self.v = v
		self.goal = goal
		self.dStop = dStop
		self.dAtObs = dAtObs                
		self.dUnsafe = dUnsafe
		self.d_fw = d_fw
		self.fwDirection = fwDirection
		self.isBlending = isBlending

		'''
		obj.p = []; % simiam.util.Plotter();
		obj.current_controller.p = obj.p;

		obj.switch_count = 0;

		'''

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
		print("VelR = " + str(velR) + " VelL = " + str(velL))
		vector = [-self.goal[0] + self.robot.stateEstimate.x, -self.goal[1] + self.robot.stateEstimate.y]
		vector[0] = vector[0]*vector[0]
		vector[1] = vector[1]*vector[1]
		val = sqrt(vector[0]+vector[1])
		if(val < 0.05):
			self.robot.setMotorsSpeeds(0,0);
			sys.exit()
		else:
			self.robot.setMotorsSpeeds(velR, velL);
		
		# [x, y, theta] = obj.state_estimate.unpack();
		# fprintf('current_pose: (%0.3f,%0.3f,%0.3f)\n', x, y, theta);

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

		xNew = x + dCenter * cos(theta)
		yNew = y + dCenter * sin(theta)
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
			print("Vlim= " + str(vLim) + " wLim= " + str(wLim))
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
			print("VELL: " + str(velL) + "VelR: " + str(velR))
		else:
			# Robot is stationary, so we can either not rotate, or
			# rotate with some minimum/maximum angular velocity
			wMin = R / L * (2 * velMin)
			wMax = R / L * (2 * velMax)
			if (abs(w) > w_min):
			    w = sign(w) * max(min(abs(w), wMax), wMin)
			else:
			    w = 0

		return self.robot.dynamics.uniToDiff(v, w)
