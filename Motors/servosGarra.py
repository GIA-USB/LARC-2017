#!/usr/bin/python
import pigpio
import time

pi = pigpio.pi()
pi.set_mode(7, pigpio.OUTPUT)
pi.set_mode(8, pigpio.OUTPUT)
while True:
	pwm = int(input("Iniciar: "))
	print("setting to: ",pi.set_servo_pulsewidth(25, 1500))
	print("setting to: ",pi.set_servo_pulsewidth(7, 1500))
	print("setting to: ",pi.set_servo_pulsewidth(8, 1500))
	
	pwm = int(input("ROTAR derecha: "))
	print("setting to: ",pi.set_servo_pulsewidth(25, 1000))
	
	pwm = int(input("Cerrar primera garra: "))
	print("setting to: ",pi.set_servo_pulsewidth(7, 1575))
	
	pwm = int(input("Rotar Izquierda: "))
	print("setting to: ",pi.set_servo_pulsewidth(25, 2000))
	print("setting to: ",pi.set_servo_pulsewidth(8, 1500))

	pwm = int(input("Cerrar segunda garra: "))
	#print("setting to: ",pi.set_servo_pulsewidth(7, 1575))
	print("setting to: ",pi.set_servo_pulsewidth(8, 1500))
	#print("setting to: ",pi.set_servo_pulsewidth(25, 2000))
	
	"""
	pwm = int(input("derecha: "))
	print("setting to: ",pi.set_servo_pulsewidth(25, 1000))
	pwm = int(input("Valor: "))
	print("setting to: ",pi.set_servo_pulsewidth(7, 1400))
	print("setting to: ",pi.set_servo_pulsewidth(8, 1600))
	print("setting to: ",pi.set_servo_pulsewidth(25, 1500))
	"""
	#time.sleep(1)
	#print("setting to: ",pi.set_servo_pulsewidth(7, pwm))
	#print("setting to: ",pi.set_servo_pulsewidth(8, pwm)) #// Mover servo
pi.stop()
