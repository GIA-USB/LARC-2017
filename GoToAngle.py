from math import sin,cos,atan2

class input:

	def __init__(self, xGoal, yGoal, v):
		self.xGoal = xGoal
		self.yGoal = yGoal
		self.v = v

class output:

	def __init__(self, v, w):
		self.v = v
		self.w = w

class GoToAngle():
	def __init__(self):
		self.checkError = 0
		self.inputs = input(0,0,0)
		self.output = output(0,0)

	def execute(self, robot, stateEstimate, inputs, dt):
		xGoal = inputs.xGoal
		yGoal = inputs.yGoal
		[x, y, theta] = stateEstimate.unpack()
		u_x = inputs.xGoal - x;     
		u_y = inputs.yGoal - y;
		thetaGoal = atan2(u_y,u_x)
		print("ThetaGoal: " + str(thetaGoal))
		error = thetaGoal - theta
		error = atan2(sin(error),cos(error))
		print("Error thetas: " + str(error))
		"""
		if (abs(error) < 1):
			if (error > 0):
				[v,w] = robot.dynamics.diffToUni(0.3,-0.3)
			else:
				[v,w] = robot.dynamics.diffToUni(-0.3,0.3)
		else:
		"""
		if (error > 0):
			[v,w] = robot.dynamics.diffToUni(0.8,-0.8)
		else:
			[v,w] = robot.dynamics.diffToUni(-0.8,0.8)
		self.checkError = error
		self.output.v = v
		self.output.w = w
		
		return self.output

	def checkTransitions(self, robot, stateEstimate, inputs):
		if (abs(self.checkError) < 0.07):
			return 2
		else:
			return 1
