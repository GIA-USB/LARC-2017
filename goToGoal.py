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

#Steers the robot towards a goal with a constant velocity using PID.
class goToGoal:

	def __init__(self, kp, ki, kd):
        # Gains.
		self.Kp = kp
		self.Ki = ki
		self.Kd = kd

        # Memory banks.
		self.errorAcum = 0
		self.errorPrev = 0

		self.inputs = input(0,0,0)
		self.output = output(0,0)
        #inputs = struct('x_g', 0, 'y_g', 0, 'v', 0);
        #outputs = struct('v', 0, 'w', 0);

	# Computes necessary linear and angular speeds that will steer the robot
    # to the goal location (x_g, y_g) with a constant linear velocity of v.
	def execute(self, robot, stateEstimate, inputs, dt):
		# Retrieve the (relative) goal location.
		xGoal = inputs.xGoal; 
		yGoal = inputs.yGoal;

		#print("xGoal: " + str(xGoal) + "yGoal: " + str(yGoal))
		# Get estimate of current pose.
		x, y, theta = stateEstimate.unpack();
		#print("x: " + str(x) + "y: " + str(y) + "Theta: " + str(theta))
		# Compute the v,w that will get you to the goal.
		v = inputs.v

		# Calculate the heading (angle) to the goal.
		# Distance between goal and robot in x-direction.
		u_x = xGoal - x;     

		# Distance between goal and robot in y-direction.
		u_y = yGoal - y;
        # Angle from robot to goal.
		thetaGoal = atan2(u_y,u_x)
		#print("u_x: " + str(u_x) + "u_y: " + str(u_y) + "Theta: " + str(thetaGoal))
		# Calculate the heading error.
        # Error between the goal angle and robot's angle.
		error = thetaGoal - theta
		print("Error antes: " + str(error))
		error = atan2(sin(error),cos(error))
		print("Error despues de atan2: " + str(error))
		if(error < 0.2):
			error = 0
  
		# Calculate PID for the steering angle.
		# Error for the integral term. Approximate the integral using the accumulated error.
		eIntegral = self.errorAcum + error * dt

		# Error for the derivative term. Approximate the derivative using the previous error.
		eDerivate = (error - self.errorPrev) / dt

		w = self.Kp * error + self.Ki * eIntegral + self.Kd * eDerivate;
		#print("W: " + str(w))
		#Save errors for next time step
		self.errorAcum = eIntegral
		self.errorPrev = error

		self.output.v = v
		self.output.w = w

		return self.output
