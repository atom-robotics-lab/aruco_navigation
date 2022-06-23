# ! /usr/bin/env python3

import imutils
import cv2
import math

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)

arucoParams = cv2.aruco.DetectorParameters_create()
vid = cv2.VideoCapture(0)


class detection():
    def __init__(self):

        self.center = None
        self.markerID1 = None
        self.radius1 = None
        self.T = 0

    def aruco_detection(self, image):

        self.image = image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        image = imutils.resize(image, width=1000)
        (corners, ids, rejected) = cv2.aruco.detectMarkers(image,
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
                
                radius = int(math.sqrt(
                    (int(topRight[0]) - int(bottomLeft[0])) ** 2 + (int(topRight[1]) - int(bottomLeft[1])) ** 2) / 2)
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(image, (cX, cY), radius, (0, 0, 255), 3)

                cv2.putText(image, "Aruco Marker ID = " + str(markerID),
                            (topLeft[0] + 20, topLeft[1] - 55),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 0, 0), 2)

              
                self.center = (((topLeft[0] + bottomRight[0]) / 2.0), ((topLeft[1] + bottomRight[1]) / 2.0))
                # print(self.center)
                self.markerID1 = markerID

                # print(self.markerID1)
                self.radius1 = radius

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            exit()

        return [gray, self.center, self.radius1, self.markerID1, image]

# if __name__== "__main__" :
#
#     det = detection()
#     det1 = det.aruco_detection(cv2.imread("/home/ayan/trying.png"))
#
#     cv2.imshow("gray" , det1[0])
#     print(det1[1])
#     cv2.waitKey(0)
