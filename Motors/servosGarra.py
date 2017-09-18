#!/usr/bin/python
import pigpio
import time

pi = pigpio.pi()
pi.set_mode(2, pigpio.OUTPUT)
while True:
	pwm = int(input("Valor: "))
	print("setting to: ",pi.set_servo_pulsewidth(2, pwm)) #// Mover servo
pi.stop()
