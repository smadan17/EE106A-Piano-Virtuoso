import numpy as np
import cv2 as cv2
import cv2.aruco as aruco
import IPython


def detect_markers(img_filepath):
	# img_filepath should be a .png
	img = cv2.imread(img_filepath)
	img = cv2.resize(img, (960, 540))
	# cv2.imshow('image', img)
	# cv2.waitKey(0)
	# cv2.destroyAllWindows()

	#parameters = aruco.DetectorParameters_create()
	marker_dict = aruco.Dictionary_get(aruco.DICT__250)
	corners, ids, rejectedImgPoints = aruco.detectMarkers(img, marker_dict)
	print(corners)
	#IPython.embed()

	for corner_set in corners:
		#corner_set is 1, 4, 2
		corner1 = corner_set[0, 0, :]
		corner2 = corner_set[0, 1, :]
		corner3 = corner_set[0, 2, :]
		corner4 = corner_set[0, 3, :]

		corner1 = np.expand_dims(corner1, axis=1)
		corner2 = np.expand_dims(corner2, axis=1)
		corner3 = np.expand_dims(corner3, axis=1)
		corner4 = np.expand_dims(corner4, axis=1)

		axis1 = corner3 - corner1
		axis2 = corner2 - corner4

		print(axis1.shape)
		A = np.hstack((axis1, -axis2))
		#IPython.embed()
		b = corner4 - corner1

		x = np.linalg.solve(A, b)

		square_center = corner4 + x[1][0]*axis2
		red = [0,0,255]
		# Change one pixel
		#IPython.embed()
		x = int(round(square_center[0][0]))
		y = int(round(square_center[1][0]))
		#print(x, y)

		cv2.circle(img,(x, y), 4, red, -1)
		#img[x,y]=red

	img_with_markers = aruco.drawDetectedMarkers(img, corners)
	cv2.imshow('image', img_with_markers) 
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def create_marker(marker_number):
	aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
	#print(aruco_dict)
	img = aruco.drawMarker(aruco_dict, marker_number, 700)
	cv2.imwrite("marker" + str(marker_number) + ".jpg", img)
	 
	cv2.imshow('frame',img)
	cv2.waitKey(0)

def main():
	#create_marker(5)
	#create_marker(100)

	#detect_markers("simin_floor_ar.jpg")
	#detect_markers("double_small.jpg")
	detect_markers("double_big.jpg")

if __name__ == "__main__":
	main()