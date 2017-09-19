from Robot import *
from Supervisor import *
from time import time, sleep
from subprocess import Popen, PIPE
from math import radians, sin, cos, atan2

# Rueda izquierda esta bien. Cuando pones IN1 High y IN2 LOW, el motor va hacia adelante.
M1IN1 = 19 # Left Motor Driver Pin M1IN1 - Purple Wire
M1IN2 = 26 # Left Motor Driver Pin M1IN2 - Orange Wire
M2IN1 = 16 # Right Motor Driver Pin M2IN1 - Purple Wire
M2IN2 = 20 # Right Motor Driver Pin M2IN2 - Orange Wire
E1A   = 12 # Right Encoder A Output - Yellow Wire
E1B   = 13 # Right Encoder B Output - White Wire
E2A   = 5  # Left Encoder A Output - Yellow Wire
E2B   = 6  # Left Encoder B Output - White Wire
'''
TRIG  = 24 
ECHO1 = 10 
ECHO2 = 9
ECHO3 = 11
ECHO4 = 25
ECHO5 = 8
ECHO6 = 7
'''
milky = Robot(0.06, 0.29, 48*74.83, 0, 180, 100, 100)
milky.setMotors(M1IN1, M1IN2, M2IN1, M2IN2)
milky.setEncoders(E1A, E1B, E2A, E2B)

#milky.setUltrasonics(TRIG,ECHO1,ECHO2,ECHO3,ECHO4,ECHO5,ECHO6)

clock = time()
milky.stateEstimate.yawOffset = 360 - milky.getYaw()
print(time() - clock)
print(milky.stateEstimate.yawOffset)
navigation = Supervisor(milky)
i = 0
oldClock = time()
newClock = 0
delta = 0
sleep(0.2)
while(True):
	newClock = time()
	delta = newClock - oldClock
	print(str(i))
	print("x = " + str(milky.stateEstimate.x) + "; y = " + str(milky.stateEstimate.y) + "; thetha = " + str(milky.stateEstimate.theta))
	print("Delta: " + str(delta))
	#yawIMU = milky.imu.getYaw()
	#print("Yaw IMU: " + str(yawIMU))
	#thetaIMU = yawIMU - yawOffset
	#print("Theta IMU: " + str(thetaIMU))
	navigation.execute(delta)
	i += 1
	oldClock = newClock
	print(str(milky.leftEncoder.ticks) + ";" + str(milky.rightEncoder.ticks))
	sleep(0.18)

pi = pigpio.pi()

pi.set_PWM_dutycycle(19, 0)
pi.set_PWM_dutycycle(26, 0)
pi.set_PWM_dutycycle(20, 0)
pi.set_PWM_dutycycle(16, 0)
pi.stop()

