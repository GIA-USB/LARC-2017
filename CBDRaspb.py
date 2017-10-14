from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import numpy as np
import cv2

# returns True if a cow is detected, False otherwise
# returns the position in the image of the cow, None otherwise
def findChessboardPosition(img, boardSize):
	# Equalizes de histogram of a grayscale image, normalize the 
	# brightness and increases the contrast of the image
	eImg = img.copy()
	eImg = cv2.cvtColor(eImg, cv2.COLOR_BGR2GRAY)
	cv2.equalizeHist(eImg, eImg)
	
	#cv2.imshow("E", eImg)
	
	# Find corners of the chessboard (cow)
	retval, corners = cv2.findChessboardCorners(eImg, boardSize, flags = cv2.CALIB_CB_ADAPTIVE_THRESH)
	
	# Draw corners found
	cv2.drawChessboardCorners(img, boardSize, corners, retval)
	
	#cv2.imshow("Chess", img)
	# Refines the corner locations: DOESNT WORKS
	#term = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
	#cv2.cornerSubPix(img, corners, boardSize, (-1, -1) , term) #criteria.maxCount
	if retval:
		(N, M) = boardSize
		i = 0
		P = [0, 0]
		for i in range(0,N*M):
			if i < N:
				P[0] = P[0] + corners[i][0][0]
			if i%3 == 0:
				P[1] = P[1] + corners[i][0][1]
				
		cv2.circle(img, (int(P[0]), int(P[1])), 1, (0, 0, 0))
		P[0] = P[0]/N
		P[1] = P[1]/M
		#cv2.imshow("Center", img)
		print(retval)
		print(str(corners))
		print(str(P))
		return retval, P
	else:
		return retval, None
		

# Cows looks like this in pictures and GIA: 3 rows x 6 columns
boardSize = (3, 4)

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# allow the camera to warmup
time.sleep(0.1)

for f in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

	frame = f.array
	
	val, position = findChessboardPosition(frame, boardSize)
	
	if (val):
		print("Cow DETECTED!")
	else:
		print("No cow detected.")

	rawCapture.truncate(0)

	if cv2.waitKey(1) == ord("z"):
		break
	
cv2.destroyAllWindows()
