
#! /usr/bin/env python3

import imutils
import cv2



arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)

arucoParams = cv2.aruco.DetectorParameters_create()

class detection ():
    def __init__(self) :

        self.center = None
        self.markerID = None

    def aruco_detection (self , image) :
            
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


        gray = imutils.resize(gray, width=1000)
        (corners, ids, rejected) = cv2.aruco.detectMarkers(gray,
            arucoDict, parameters=arucoParams)
        if len(corners) > 0:
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                cv2.line(gray, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(gray, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(gray, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(gray, bottomLeft, topLeft, (0, 255, 0), 2)
                
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(gray, (cX, cY), 10, (312,0,0), -3)
                
                cv2.putText(gray, str(markerID),
                    (topLeft[0], topLeft[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
                self.center = [cX , cY]
                self.markerID = markerID

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            exit()
            

        return [ gray , self.center , self.markerID ]
            
if __name__== "__main__" :

    det = detection()
    det1 = det.aruco_detection(cv2.imread("/home/ayan/trying.png"))

    cv2.imshow("gray" , det1[0])
    cv2.waitKey(0)

