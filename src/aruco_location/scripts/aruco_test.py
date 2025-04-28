#! /usr/bin/env python3

# Package Name: aruco_location

import cv2

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

img = cv2.imread("2.png")
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# cv2.imshow("image",img_gray)
# cv2.waitKey(0)

corners, ids, _ = detector.detectMarkers(img_gray)
print("corners: ", corners)
print("ids: ", ids)

cv2.destroyAllWindows()
