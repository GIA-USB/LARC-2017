import pigpio
import time

PIN = 3

pi = pigpio.pi()
pi.set_mode(PIN, pigpio.OUTPUT)

veces = 4
pwm = 0
while True:
    print("setting to: ", pwm)
    pi.set_servo_pulsewidth(PIN,pwm)
    try:
        pwm = int(input("Metelo: "))
    except:
        break
    print("setting to: ", pwm)
    pi.set_servo_pulsewidth(PIN,pwm)
    try:
        pwm = int(input("Metelo: "))
    except:
        break

pi.set_servo_pulsewidth(PIN,0)
