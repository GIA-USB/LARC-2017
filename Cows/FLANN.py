from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import cv2
import time
from matplotlib import pyplot as plt

camera = PiCamera()
camera.resolution = (320, 240)
camera.framerate = 32
# Crop takes values between 0 and 1
camera.zoom = (0.5, 0.5, 0.5, 0.5)
camera.brightness = 55
rawCapture = PiRGBArray(camera, size=(320, 240))

time.sleep(0.1)

img2 = cv2.imread('testVaca.png',3)  # trainImage
#img1 = cv2.imread('vacas.png', 3)

for f in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		
	img1 = f.array # queryImage

	img1 = cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)
	img2 = cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)

	# Initiate SIFT detector
	sift = cv2.xfeatures2d.SIFT_create(nOctaveLayers = 5, contrastThreshold = 0.04, edgeThreshold = 20,
								   sigma = 1.6)

	# find the keypoints and descriptors with SIFT
	kp1, des1 = sift.detectAndCompute(img1,None)
	kp2, des2 = sift.detectAndCompute(img2,None)

	# FLANN parameters
	FLANN_INDEX_KDTREE = 0
	index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
	search_params = dict(checks=2000)   # or pass empty dictionary
	
	# FLANN based descriptor matcher of the query and train
	flann = cv2.FlannBasedMatcher(index_params,search_params)

	# returns k best matches
	matches = flann.knnMatch(des1,des2,k=2)

	#print(des1)
	# Need to draw only good matches, so create a mask
	matchesMask = [[0,0] for i in range(len(matches))]

	# ratio test as per Lowe's paper
	for i,(m,n) in enumerate(matches):
		#print("m : " + str(m) + "n : " + str(n) )
		if m.distance < 0.7*n.distance:
			matchesMask[i]=[1,0]

	#print("Matched: " + str(len(matchesMask)))
	draw_params = dict(matchColor = (0,255,0),
						singlePointColor = (255,0,0),
						matchesMask = matchesMask,
						flags = 0)
	
	# find common points and draw them in the image output
	img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,matches,None,**draw_params)

	plt.imshow(img3,),plt.show()

	rawCapture.truncate(0)

#if cv2.waitKey(1) == ord("z"):
#	break
