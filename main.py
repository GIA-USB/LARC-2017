from Robot import *
from Supervisor import *
import time
from time import sleep

# Rueda izquierda esta bien. Cuando pones IN1 High y IN2 LOW, el motor va hacia adelante.
M1IN1 = 19 # Left Motor Driver Pin M1IN1 
M1IN2 = 26 # Left Motor Driver Pin M1IN2
M2IN1 = 16 # Right Motor Driver Pin M2IN1
M2IN2 = 20 # Right Motor Driver Pin M2IN2
E1A   = 12 # Right Encoder A Output - Yellow Wire
E1B   = 13 # Right Encoder B Output - White Wire
E2A   = 5  # Left Encoder A Output - Yellow Wire
E2B   = 6  # Left Encoder B Output - White Wire
TRIG  = 24 
ECHO1 = 10 
ECHO2 = 9
ECHO3 = 11
ECHO4 = 25
ECHO5 = 8
ECHO6 = 7

milky = Robot(0.06, 0.29, 48*74.83, 0, 180, 100, 100)
milky.setMotors(M1IN1, M1IN2, M2IN1, M2IN2)
milky.setEncoders(E1A, E1B, E2A, E2B)
milky.setIMU()
milky.setUltrasonics(TRIG,ECHO1,ECHO2,ECHO3,ECHO4,ECHO5,ECHO6)
#milky.imu.getData(2000)
#print(milky.imu)
clock = time.time()
print(milky.imu.getFirstOrientation(250))
print(time.time() - clock)
navigation = Supervisor(milky)
i = 0
oldClock = time.time()
newClock = 0
delta = 0
sleep(0.2)
while(False):
	newClock = time.time()
	delta = newClock - oldClock
	print(str(i))
	print("x = " + str(milky.stateEstimate.x) + "; y = " + str(milky.stateEstimate.y) + "; thetha = " + str(milky.stateEstimate.theta))
	print("Delta: " + str(delta))
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

