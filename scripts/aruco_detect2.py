import cv2
import cv2.aruco as aruco
import math
class aruco_detection:
    def __init__(self):
        self.markersize=6
        self.totalmarkers=250

    def circle_draw(self,m,n):
        print(m)
        print(n)
        x=(m[0]+n[0])/2
        y=(m[1]+n[1])/2
        radius=math.sqrt((n[0]-m[0])**2+(n[1]-m[1])**2)/2
        wt = (x, y, radius)
        return wt
    
    def findArucoMarkers(self,img):
        self.img=img
        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        key = getattr(aruco, f'DICT_{self.markersize}X{self.markersize}_{self.totalmarkers}')
        arucoDict = aruco.Dictionary_get(key)
        arucoParam = aruco.DetectorParameters_create()
        (bboxs, ids, rejected) = aruco.detectMarkers(gray, arucoDict, parameters = arucoParam)
        print(bboxs)
        x,y,radius=self.circle_draw(bboxs[0][0][0],bboxs[0][0][2])
        print('x:',x,'y:',y,'radius:',radius)
        cv2.circle(self.img,(int(x),int(y)),int(radius),(312,0,0),3)
        aruco.drawDetectedMarkers(img, bboxs)
        cv2.imshow('img',self.img)

        k=cv2.waitKey(0) & 0xff
        if k==27:
            exit()
        return(self.img,(x,y),radius)


if __name__=="__main__":
    det=aruco_detection()
    img=cv2.imread("aruco1.png")
    det.findArucoMarkers(img)
    

   
