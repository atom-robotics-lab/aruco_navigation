
#! /usr/bin/env python3

import cv2
import math
class aruco_detection:
    def __init__(self):
        self.markersize=4
        self.totalmarkers=250
        self.markerID = None
        self.radius =  None

    def circle_draw(self,m,n):
        print(m)
        print(n)
        x=(m[0]+n[0])/2
        y=(m[1]+n[1])/2
        radius=math.sqrt((n[0]-m[0])**2+(n[1]-m[1])**2)/2
        wt = (x, y, radius)
        return wt
    
    def findArucoMarkers(self, image):
        self.img=image

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        key = getattr(cv2.aruco , f'DICT_{self.markersize}X{self.markersize}_{self.totalmarkers}')
        arucoDict = cv2.aruco.Dictionary_get(key)
        arucoParam = cv2.aruco.DetectorParameters_create()

        (bboxs, ids, rejected) = cv2.aruco.detectMarkers(gray, arucoDict, parameters = arucoParam)
        print(bboxs)
       

        end_result = self.circle_draw(bboxs[0][0][0],bboxs[0][0][2])
        x = end_result[0]
        y = end_result[1]
        radius = end_result[2]
        print('x:',x,'y:',y,'radius:',radius)
        cv2.circle(self.img,(int(x),int(y)),int(radius),(312,0,0),3)
        cv2.aruco.drawDetectedMarkers(img, bboxs)
        self.markerID = ids
        self.radius = radius

        cv2.imshow('img',self.img)


        k=cv2.waitKey(0) & 0xff
        if k==27:
            exit()
        return(self.img,(x,y),self.radius , self.markerID)


if __name__=="__main__":
    det=aruco_detection()
    img=cv2.imread("/home/ayan/trying.png")
    detection = det.findArucoMarkers(img)
    

   