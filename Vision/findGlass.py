from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import cv2
import time

def getContours(img):
	gray = img
	if(len(img.shape) > 2):
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	#kernel = np.ones((5,5),np.uint8)
	#gray = cv2.erode(gray,kernel,iterations = 1)
	#gray = cv2.medianBlur(gray, 9)
	thresh = cv2.adaptiveThreshold(	gray.copy(),
									255,
									cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
										cv2.THRESH_BINARY,11,2)
	_, contours, hierarchy =   cv2.findContours(thresh.copy(),
											 cv2.RETR_LIST,
											 cv2.CHAIN_APPROX_SIMPLE)
	return contours, hierarchy

def boundingBoxes(img, contours):
	count = 0
	for c in contours:
		area = cv2.contourArea(c)	# Area del contorno
		M = cv2.moments(c)			# Moments del contorno.
		if(area > 300):
			x,y,w,h = cv2.boundingRect(c)
			cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)	# Dibujar rectangulo
			count += 1
	return count

def reduceValue(image, darkness):
	result = np.array(image)
	dims = image.shape
	for i in range(dims[0]):
		for j in range(dims[1]):
			if((result[i][j][2] - darkness)<0):
				result[i][j][2] = 0
			else:
				result[i][j][2] = result[i][j][2] - darkness
	return result
		
def findGlassHole(camera, delay):
	# return -> 1: Meter garra 0: No hay vaso
	# Recibe la imagen en bgr
	
	"""
	Preprocesamiento
		Resize, erosiones, blurs, brillos, espacios de colores.
		RESIZE
			height, width = img.shape[:2]	# Resize de la imagen para 
			img = cv2.resize(img,(width/4, height/4), interpolation = cv2.INTER_CUBIC)
		COLOR SPACE
			img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	"""
	camera.resolution = (320, 240)
	camera.framerate = 32
	rawCapture = PiRGBArray(camera, size=(320, 240))
	# time.sleep(0.1)

	time_start = time.time()
	bool_visto = False
	aux = 1
	for f in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True): 
		img = f.array
		kernel = np.ones((3,3),np.uint8)
		img = cv2.erode(img,kernel,iterations = 1)
		original = img.copy()

		# Rango de deteccion de colores. 
			#GREEN: (50,160,50) (255,200,255) HSV
			#ROJO: (50, 50, 225) (255,255,255) HSV
		upper = (255, 255, 255)
		lower = (200, 200,200 )
		mask = cv2.inRange(img, lower, upper)
		img = cv2.bitwise_and(img, img, mask = mask)

		contours, hier = getContours(img)		# Obtener contornos y jerarquias.
		count = boundingBoxes(img, contours)	# Dibujar cajas.

		if time.time() - time_start > delay:	# Si la garra lleg'o al final, chao.
			return 0
		if count > 1:
			bool_visto = True
		elif bool_visto and count<=1:
			print("METE LA GARRA HASTA EL FONDO", aux)
			aux += 1


			return 1;
		if count <= 1:
			bool_visto=False
		
		# Mostrar cositas para debug
		cv2.imshow("Image1", original)
		cv2.imshow("Image", img)
		rawCapture.truncate(0)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			cv2.destroyAllWindows()
			break
