import cv2
import cv2.aruco as aruco

def findArucoMarkers(img, markerSize = 6, totalMarkers=250, draw=True):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    key = getattr(aruco, f'DICT{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters = arucoParam)
    print(ids)
    if draw:
        aruco.drawDetectedMarkers(img, bboxs)

image = cv2.imread('ss2.png')
findArucoMarkers(image)
cv2.imshow('image',image)