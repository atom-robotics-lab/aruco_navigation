#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import os


def publish_message():
    def FindArucoMarker(img, markerSize=5, totalMarkers=1000, draw=True):
        imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # key = getattr(aruco,f'DICT_5X5_1000')
        arucoDict = aruco.Dictionary_get(aruco.DICT_5X5_1000)
        arucoParam = aruco.DetectorParameters_create()
        bboxs, ids, rejected = aruco.detectMarkers(imgGray, arucoDict, parameters=arucoParam)

        if draw:
            aruco.drawDetectedMarkers(img, bboxs)
        return [ids, bboxs]




    #cap = cv2.VideoCapture(1)
    pub = rospy.Publisher('video_frames', Image, queue_size=10)


    rospy.init_node('video_pub_py', anonymous=True)

    rate = rospy.Rate(10)
    img = cv2.imread('aruco.png')
    while True:
        
        arucoFound = FindArucoMarker(img)
        cv2.imshow("Image", img)

        print(arucoFound)
        # cv2.imshow("Image", img)
        if cv2.waitKey(27) & 0xFF == ord('q'):
            break
  
    #br = CvBridge()
    #rospy.loginfo('publishing video frame')


    #pub.publish(br.cv2_to_imgmsg(frame))




        

    

       

    # Sleep just enough to maintain the desired rate
    rate.sleep()

if __name__ == '__main__':
    try:
        publish_message()
    except rospy.ROSInterruptException:
        pass
