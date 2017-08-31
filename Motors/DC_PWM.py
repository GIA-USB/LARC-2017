import RPi.GPIO as IO

import time

IO.setwarnings(False)

x = 0

IO.setmode(IO.BCM)

IO.setup(13,IO.OUT)
IO.setup(19,IO.OUT)

p1 = IO.PWM(13, 100)
p2 = IO.PWM(19, 100)

p1.start(0)
p2.start(0)

x = 75

'''
while 1:

	if ( x < 0 ):
		p1.ChangeDutyCycle(x)
		x = x-1
		time.sleep(0.2)
	
	else:
		break
		
while 1:

	if ( x <= 75 ):
		p2.ChangeDutyCycle(x)
		x = x+1
		time.sleep(0.2)
	
	else:
		break
'''
i = 0
while i < 1000000:
	p1.ChangeDutyCycle(100)
	i += 1
p1.ChangeDutyCycle(0)
