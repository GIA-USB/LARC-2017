#!/usr/bin/python
import pigpio
import time

ORDENHA = 4
APRIETA = 3

pi = pigpio.pi()
pi.set_mode(ORDENHA, pigpio.OUTPUT)
pi.set_mode(APRIETA, pigpio.OUTPUT)

#print("setting to: ",pi.set_servo_pulsewidth(APRIETA, 1000)) # Mover servo
#print("set to: ",pi.get_servo_pulsewidth(APRIETA))

#time.sleep(1)
veces = 4
for i in range(veces):

    print("setting to: ",pi.set_servo_pulsewidth(APRIETA, 1625)) 
    print("set to: ",pi.get_servo_pulsewidth(APRIETA))
    #for i in range(8):
    #    pi.set_servo_pulsewidth(APRIETA,1450)
    #    time.sleep(0.5)
    #    pi.set_servo_pulsewidth(APRIETA,1550)
        #print("setting to: ",pi.set_servo_pulsewidth(ORDENHA, 1000)) 
    #    time.sleep(1.5)

    time.sleep(4)
    
    pi.set_servo_pulsewidth(ORDENHA,1700)
    time.sleep(2)
    print("setting to: ",pi.set_servo_pulsewidth(ORDENHA, 0))
    time.sleep(3)
    print("setting to: ",pi.set_servo_pulsewidth(APRIETA, 1420)) # Detener servo (Hace torque)
    time.sleep(3)

print("setting to: ",pi.set_servo_pulsewidth(ORDENHA, 0))    # Detener servo (Sin torque)
print("set to: ",pi.get_servo_pulsewidth(ORDENHA))

print("setting to: ",pi.set_servo_pulsewidth(APRIETA, 0))    # Detener servo (Sin torque)
print("set to: ",pi.get_servo_pulsewidth(APRIETA))
pi.stop()
