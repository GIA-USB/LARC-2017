#!/usr/bin/python
import pigpio
import time

# Variables.
motor_d1 = 19
motor_d2 = 26
motor_i1 = 16
motor_i2 = 20

# Acercarse un threshold
const_threshold = 3

pi = pigpio.pi()
pi.set_mode(motor_d1, pigpio.OUTPUT)
pi.set_mode(motor_d2, pigpio.OUTPUT)
pi.set_mode(motor_i1, pigpio.OUTPUT)
pi.set_mode(motor_i1, pigpio.OUTPUT)

def forward(pwm):
	pi.set_PWM_dutycycle(motor_d1, pwm)
	pi.set_PWM_dutycycle(motor_i2, pwm)

def backward(pwm):
	pi.set_PWM_dutycycle(motor_d2, pwm)
	pi.set_PWM_dutycycle(motor_i1, pwm)

def stop():
	pi.set_PWM_dutycycle(motor_d1, 0)
	pi.set_PWM_dutycycle(motor_d2, 0)
	pi.set_PWM_dutycycle(motor_i1, 0)
	pi.set_PWM_dutycycle(motor_i2, 0)

def turnLeft(pwm):
	pi.set_PWM_dutycycle(motor_d1, pwm)
	pi.set_PWM_dutycycle(motor_i1, pwm)

def turnRight(pwm):
	pi.set_PWM_dutycycle(motor_d2, pwm)
	pi.set_PWM_dutycycle(motor_i2, pwm)

#pi.stop()


