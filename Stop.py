class input:

	def __init__(self, xGoal, yGoal, v):
		self.xGoal = xGoal
		self.yGoal = yGoal
		self.v = v

class output:

	def __init__(self, v, w):
		self.v = v
		self.w = w

class Stop():
	def __init__(self):
		self.inputs = input(0,0,0)
		self.output = output(0,0)

	def execute(self, robot, stateEstimate, inputs, dt):
		self.output.v = 0
		self.output.w = 0
		
		return self.output

	def checkTransitions(self, robot, stateEstimate, inputs):
		return 0
