import numpy as np
import cv2 as cv2
import cv2.aruco as aruco


def detect_markers(img_filepath):
    # img_filepath should be a .png
    img = cv2.imread(img_filepath)

    arr = [
        [0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 0, 1, 1, 0],
        [0, 1, 1, 0, 1, 1, 0],
        [0, 1, 0, 1, 0, 1, 0],
        [0, 0, 1, 1, 1, 0, 0],
        [0, 0, 1, 1, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0],
    ]
    arr_flat = []
    for a in arr:
        for b in a:
            arr_flat.append(b)
    mat = np.array(arr_flat)
    bytesList = aruco.Dictionary_getByteListFromBits(mat)
    marker_dict = aruco.custom_dictionary(1, 7)
    marker_dict.bytesList = bytesList
    # marker_dict.maxCorrectionBits = 10
    img1 = aruco.drawMarker(marker_dict, 0, 500)
    cv2.imwrite("mtest.jpg", img1)

    #parameters = aruco.DetectorParameters_create()
    # marker_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
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

    # img = cv2.resize(img, (int(img.shape[0]/4), int(img.shape[1]/4)))
    img_with_markers = aruco.drawDetectedMarkers(img, corners)
    cv2.imwrite("btest.jpg", img_with_markers)
    cv2.imshow('image', img_with_markers)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def main():
    # detect_markers("alvar.jpg")
    detect_markers("alvar.jpg")

if __name__ == "__main__":
    main()
