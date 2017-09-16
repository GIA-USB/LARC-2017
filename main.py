from Robot import *
from Supervisor import *
import time
from time import sleep

M1IN1 = 19 # Motor Driver Pin M1IN1 
M1IN2 = 26 # Motor Driver Pin M1IN2
M2IN1 = 16 # Motor Driver Pin M2IN1
M2IN2 = 20 # Motor Driver Pin M2IN2
E1A   = 12 # Right Encoder A Output - Yellow Wire
E1B   = 13 # Right Encoder B Output - White Wire
E2A   = 5  # Left Encoder A Output - Yellow Wire
E2B   = 6  # Left Encoder B Output - White Wire

milky = Robot(0.06, 0.29, 48*74.83, 0, 180, 100, 100)
milky.setMotors(M1IN1, M1IN2, M2IN1, M2IN2)
milky.setEncoders(E1A, E1B, E2A, E2B)
navigation = Supervisor(milky)
i = 0
oldClock = time.time()
newClock = 0
delta = 0
while(True):
	newClock = time.time()
	delta = newClock - oldClock
	print(str(i))
	print("x = " + str(milky.stateEstimate.x) + "; y = " + str(milky.stateEstimate.y) + "; thetha = " + str(milky.stateEstimate.theta))
	navigation.execute(delta)
	i += 1
	oldClock = newClock
	#print(str(milky.leftEncoder.ticks) + ";" + str(milky.rightEncoder.ticks))
	sleep(0.2)

pi = pigpio.pi()

pi.set_PWM_dutycycle(19, 0)
pi.set_PWM_dutycycle(26, 0)
pi.set_PWM_dutycycle(20, 0)
pi.set_PWM_dutycycle(16, 0)
pi.stop()

