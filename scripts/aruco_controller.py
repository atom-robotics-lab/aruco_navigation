#! /usr/bin/env python3
from re import X
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
# from anushkacv import findArucoMarkers
from opencv11 import detection
from cv_bridge import CvBridge, CvBridgeError

import cv2
from sensor_msgs.msg import Image

class Robot_Controller:
    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw",Image,self.callback)
        self.image_pub = rospy.Publisher("image_topic_2",Image)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        
        self.pose = []
        self.state = 0
        self.velocity_msg = Twist()
        self.p = 0.01
        self.at = 0.00001
        self.radius_threshold = 90

    



    def move(self,linear,angular):
        self.velocity_msg.linear.x = linear
        self.velocity_msg.angular.z = angular 
        self.pub.publish(self.velocity_msg)

    def callback(self,data):
        try:
            self.cv1_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image_showing()
        except CvBridgeError as e :
            print(e)
            
    def image_showing (self):   

        theta_precision = 30
        dist_precision = 3.0
        detect = detection()
        detection_result = detect.aruco_detection(self.cv1_image)  
        
      


        if  ( detection_result[3] == None )  :
          self.move(0 , 0.5)
          rospy.loginfo("findng aruco")

        else :

          # cv2.imshow("detected" , detection_result[0])
          x_length = detection_result[0].shape[0]
          aruco_postion = detection_result[1][0]
          theta_error1 = - aruco_postion + int(x_length/2)
          self.theta_error = theta_error1
          rospy.loginfo("initial theta error" +str(self.theta_error))
          rospy.loginfo("initial radius " +str(detection_result[2]))

          if  ( detection_result[2] < self.radius_threshold )  :

            rospy.loginfo("radius after first one " +str(detection_result[2]))

            if (  theta_precision <  abs(theta_error1) ) and (theta_error1 > 0 ) :

              self.move (0.3 , -1*self.at*self.theta_error)
              # print(self.theta_error)
              rospy.loginfo("Theta_error" +str(self.theta_error))

            elif (theta_precision < abs(theta_error1)) and (theta_error1 < 0 ) :

              self.move(0.3, self.at*self.theta_error )
              rospy.loginfo("second_error" + str(self.theta_error))
              # print(  self.theta_error )

            else :

              # self.move (self.at*(-self.radius_threshold + detection_result[2] ), 0 )
              self.move(0.5 , 0)
              # rospy.loginfo("moving straight")
              print("moving straight")

          elif detection_result[2] >= self.radius_threshold :
            
            rospy.loginfo("radius afterwards" +str(detection_result[2]))

            if (  int(x_length)/2 < (aruco_postion))  :



              self.move (0 , -1*self.p*self.theta_error)
              rospy.loginfo("Theta_error 3" + str(self.theta_error))

            elif  (  int(x_length)/2 > (aruco_postion))  :

              self.move(0 , self.p*self.theta_error )
              # rospy.loginfo("second_error 3" , str(self.theta_error))
            else :

              self.move(0, 0)
              # rospy.loginfo("bot reached")

        cv2.imshow("gray" , detection_result[0])
        cv2.waitKey(1)
def main():
  rospy.init_node("robot controller",anonymous=True)
  of=Robot_Controller()
  try:
    rospy.spin()
    
  except:
    print("error")


  cv2.destroyAllWindows()


main()

    