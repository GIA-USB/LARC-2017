#!/usr/bin/python
import pigpio
import time
#
pi = pigpio.pi()
pi.set_mode(19, pigpio.OUTPUT)
pi.set_mode(26, pigpio.OUTPUT)

pi.set_mode(16, pigpio.OUTPUT)
pi.set_mode(20, pigpio.OUTPUT)

pi.set_PWM_dutycycle(19, 255)
pi.set_PWM_dutycycle(26, 0)

pi.set_PWM_dutycycle(16, 0)
pi.set_PWM_dutycycle(20, 255)
time.sleep(3)

pi.set_PWM_dutycycle(19, 0)
pi.set_PWM_dutycycle(26, 0)
pi.set_PWM_dutycycle(20, 0)
pi.set_PWM_dutycycle(16, 0)
pi.stop()


