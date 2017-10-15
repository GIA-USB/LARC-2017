import pigpio
import time

PIN1 = 3
PIN2 = 4

pi = pigpio.pi()
pi.set_mode(PIN1, pigpio.OUTPUT)
pi.set_mode(PIN2, pigpio.OUTPUT)

while True:
	
    pi.set_servo_pulsewidth(PIN1,500)
    time.sleep(1)
    pi.set_servo_pulsewidth(PIN2,1000)
    time.sleep(1)
    pi.set_servo_pulsewidth(PIN1, 0)
    pi.set_servo_pulsewidth(PIN2, 0)
    time.sleep(2)
    pi.set_servo_pulsewidth(PIN1, 2000)
    pi.set_servo_pulsewidth(PIN2, 2000)
    time.sleep(0.2)
    pi.set_servo_pulsewidth(PIN1, 0)
    pi.set_servo_pulsewidth(PIN2, 0)
    time.sleep(4)
