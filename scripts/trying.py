#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
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
        self.at = 0.01
        self.radius_threshold = 150

    



    def move(self,linear,angular):
        self.velocity_msg.linear.x = linear
        self.velocity_msg.angular.z = angular 
        self.pub.publish(self.velocity_msg)

    def callback(self,data):
        try:
            self.cv1_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            print("callback")
            self.image_showing()
        except CvBridgeError as e :
            print(e)
            
    def image_showing (self):   


      theta_precision = 30
      detect = detection()
      detection_result = detect.aruco_detection(self.cv1_image)  

      x_length = detection_result[0].shape[0]
      # aruco_postion = detection_result[1][0]
      theta_error1 = 0 
      self.theta_error = theta_error1

      while True:
        if detection_result[3] != None :

          aruco_postion = detection_result[1][0]

          theta_error1 = int(x_length)/2 - aruco_postion 
          self.theta_error = theta_error1
          if detection_result[2] < self.radius_threshold :

            if ( theta_precision < abs(theta_error1) ) and ( theta_error1 > 0 ) :
              self.move(0.3 , -self.at*self.theta_error)
              print("left")


            elif ( theta_precision < abs (theta_error1) ) and ( theta_error1 < 0 ) :
              self.move(0.3 , self.at*self.theta_error)
              print("right")
            else :
              self.move(0.3, 0)

          else :
            self.move(0,0)
            print("bot sucks")
        else :
          self.move(0 ,0.3)
          print("finding suiiii")
        cv2.imshow("gray" , detection_result[0])
        cv2.waitKey(1)
def main():
  rospy.init_node("robot controller",anonymous=True)
  of=Robot_Controller()
  print("lalalala")
  try:
    print("nananana")
    rospy.spin()

    
  except:
    print("error")


  cv2.destroyAllWindows()


main()
        