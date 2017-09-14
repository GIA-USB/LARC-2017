import numpy as np
import cv2

MIN_MATCH_COUNT = 10
FLANN_INDEX_KDTREE = 0

def matchAndFind(imgQuery, kpQuery, desQuery, imgTrain, kpTrain, desTrain):
	detect = False
	matches = flann.knnMatch(desQuery,desTrain,k=2)

	# Store all the good matches as per Lowe's ratio test.
	good = []
	for m,n in matches:
	    if m.distance < 0.7 * n.distance:
	        good.append(m)
	print("Number of good matches: " + str(len(good)))

	if len(good) > MIN_MATCH_COUNT:
		try:
		    src_pts = np.float32([ kpQuery[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
		    dst_pts = np.float32([ kpTrain[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

		    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
		    matchesMask = mask.ravel().tolist()

		    h,w = imgQuery.shape
		    pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
		    dst = cv2.perspectiveTransform(pts,M)

		    imgTrain = cv2.polylines(imgTrain,[np.int32(dst)],True,255,3, cv2.LINE_AA)
		    detect = True
		except:
			print("Exception ocurred.")		    
	else:
		#print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
		matchesMask = None
	draw_params = dict(matchColor = (0,255,0), # draw matches in green color
	                   singlePointColor = None,
	                   matchesMask = matchesMask, # draw only inliers
	                   flags = 2)
	img3 = cv2.drawMatches(imgQuery,kpQuery,imgTrain,kpTrain,good,None,**draw_params)
	cv2.imshow("All", img3)
	return detect

imgTerraine = cv2.imread('Marcas/Marca Kanji.png',0) # Empty terraines zone marker 
imgExchange = cv2.imread('Marcas/Marca L.png',0) # Exchange zone marker
imgTank = cv2.imread('Marcas/marker.png',0) # Milk tank marker

# Initiate SIFT detector
sift = cv2.xfeatures2d.SIFT_create(nOctaveLayers = 5, contrastThreshold = 0.04, edgeThreshold = 10,
								   sigma = 1.6)

# Find the keypoints and descriptors with SIFT
kpTerraine, desTerraine = sift.detectAndCompute(imgTerraine,None)
kpExchange, desExchange = sift.detectAndCompute(imgExchange,None)
kpTank, desTank = sift.detectAndCompute(imgTank,None)

index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks = 50)

flann = cv2.FlannBasedMatcher(index_params, search_params)

cap = cv2.VideoCapture(0)
while(cap.isOpened()):
	ret, frame = cap.read()
	frame = cv2.flip(frame, 1)
	kpFrame, desFrame = sift.detectAndCompute(frame,None)
	if (matchAndFind(imgExchange, kpExchange, desExchange, frame, kpFrame, desFrame)):
		print("Exchange zone DETECTED!")
	elif (matchAndFind(imgTerraine, kpTerraine, desTerraine, frame, kpFrame, desFrame)):
		print("Empty terraines zone DETECTED!")
	elif (matchAndFind(imgTank, kpTank, desTank, frame, kpFrame, desFrame)):
		print("Milk tank zone DETECTED!")
	else:
		print("No zone detected.")

	if cv2.waitKey(1) == ord("z"):
		break
cap.release()		
cv2.destroyAllWindows()
	
