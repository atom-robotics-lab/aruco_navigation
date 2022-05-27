import cv2
import cv2.aruco as aruco
import numpy as np
import os
import math



class ArucoMarker :
    def __init__(self) :
        self.markerSize = 6
        self.totalMarkers = 250

    def find_circle(self , p1 , p2):
        print("p1 : ",p1,"p2 : ",p2)
        x = ( p2[0] - p1[0] )/2 + p1[0]
        y = ( p2[1] - p1[1] )/2 + p1[1]
        r = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)/2
        return int(x),int(y),int(r)

    def find_aruco(self , img) :
        self.img = img

        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        key = getattr(aruco, f'DICT_{self.markerSize}X{self.markerSize}_{self.totalMarkers}')
        arucoDict = aruco.Dictionary_get(key)
        arucoParam = aruco.DetectorParameters_create()
        
        bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters = arucoParam)
        print(bboxs)
        #aruco.drawDetectedMarkers(self.img, bboxs)

        x,y,r = self.find_circle(bboxs[0][0][0] , bboxs[0][0][2])
        print("x : ",x,"y : ",y,"r : ",r)
        
        
        cv2.circle(self.img , (x,y) , r , (255, 0, 0) , 2)
        #cv2.circle(self.img , (150,150) , 50 , (255, 0, 0) , 2)
            
        

        cv2.imshow('img',self.img)
        k = cv2.waitKey(0) & 0xff
        if k == 27:
            exit()
        
        return [self.img , (x,y) ,r]


if __name__ == "__main__" :
    am = ArucoMarker()
    img = cv2.imread("aruco2.png")

    am.find_aruco(img)

        





