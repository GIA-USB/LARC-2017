import subprocess, signal
import os
import sys
import time
import termios

from math import radians
import numpy as np

class IMU():
	def __init__(self):
		pass
		
	def getFirstYaw(self,iteraciones):
		i = 0
		fusionPose = np.array([0,0,0])
		process = subprocess.Popen(['minimu9-ahrs --output euler'], stdout=subprocess.PIPE, shell=True)
		while(i < 2*iteraciones):
			print("Iteracion: " + str(i))
			out = process.stdout.readline()
			if (out == '' and process.poll() != None):
				break
			if (out != ''):
				data = out.split()
				data = np.array([float(data[0]),float(data[1]),float(data[2])])
				if (i >= iteraciones):
					fusionPose = fusionPose + data
				i += 1
				print(data)
		fusionPose /= iteraciones
		#fusionPose[2] = math.degrees(fusionPose[2])
		print(fusionPose[0])
		return radians(fusionPose[0])
			
	def getYaw(self):
		process = subprocess.Popen(['minimu9-ahrs --output euler'], stdout=subprocess.PIPE, shell=True)
		out = process.stdout.readline()
		if (out == '' and process.poll() != None):
			pass
		if (out != ''):
			data = out.split()
			data = [float(data[0]),float(data[1]),float(data[2])]
		print(data[0])
		return radians(data[0])
