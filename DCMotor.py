import pigpio

class Motor:
	def __init__(self, pi, in1, in2):
		self.pi = pi
		self.IN1 = in1
		self.IN2 = in2
		
		self.pi.set_mode(in1, pigpio.OUTPUT)
		self.pi.set_mode(in2, pigpio.OUTPUT)
	
	def setMotorPWM(self, pwm1, pwm2):
		print("IN1: " + str(self.IN1))
		print("IN2: " + str(self.IN2))
		print("PWM1: " + str(pwm1))
		print("PWM2: " + str(pwm2)) 
		self.pi.set_PWM_dutycycle(self.IN1, pwm1)
		self.pi.set_PWM_dutycycle(self.IN2, pwm2)
