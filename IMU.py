import subprocess, signal
import os
import sys
import time
import termios

import math
import numpy as np

class IMU():
	def __init__(self):
		pass
		
	def getFirstOrientation(self,iteraciones):
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
		return fusionPose
			
	def getData(self,iteraciones):
		i = 0
		fusionPose = np.array([0,0,0])
		process = subprocess.Popen(['minimu9-ahrs --output euler'], stdout=subprocess.PIPE, shell=True)
		while (True):
			print("Iteracion: " + str(i))
			out = process.stdout.readline()
			if (out == '' and process.poll() != None):
				break
			if (out != ''):
				data = out.split()
				data = [float(data[0]),float(data[1]),float(data[2])]
			i += 1
		return data
