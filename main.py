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
GarI  = 8  # Motor garra derecha - Green Wire
GarD  = 7  # Motor garra izquierda - Green Wire
GRot  = 25 # Motor que rota las garras - Green wire
TRIG  = 27 # Trigger pin for all ultrasonics - White Wire
ECHO1 = 24 # Echo pin for Right US - Purple Wire
ECHO2 = 23 # Echo pin for Center US - Purple Wire
ECHO3 = 22 # Echo pin for Left US - Purple Wire

milky = Robot(0.06, 0.29, 48*74.83, 0, 180, 100, 100)
milky.setMotors(M1IN1, M1IN2, M2IN1, M2IN2)
milky.setEncoders(E1A, E1B, E2A, E2B)
milky.setIMU()
milky.setUltrasonics(TRIG,ECHO1,ECHO2,ECHO3)

clock = time()
milky.stateEstimate.yawOffset = milky.imu.getFirstYaw(50)
print(time() - clock)
print("CENTRO: ", milky.stateEstimate.yawOffset)
navigation = Supervisor(milky)
i = 0
oldClock = time()
newClock = 0
delta = 0
while(True):
	distances = milky.getUSDistances()
	print("Distancias: ",distances)
sleep(0.2)
while(False):
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
	sleep(0.1)

pi = pigpio.pi()

pi.set_PWM_dutycycle(19, 0)
pi.set_PWM_dutycycle(26, 0)
pi.set_PWM_dutycycle(20, 0)
pi.set_PWM_dutycycle(16, 0)
pi.stop()

