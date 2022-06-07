#! /usr/bin/env python3
from turtle import distance, position
import rospy
from geometry_msgs.msg import Point,Twist
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion
from opencv import detection
from cv_bridge import CvBridge, CvBridgeError

import cv2
from sensor_msgs.msg import Image

class Robot_Controller:
    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw",Image,self.callback)
        self.image_pub = rospy.Publisher("image_topic_2",Image)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.pose = []
        self.state = 0
        self.velocity_msg = Twist()
        self.p = 0.5
        self.at = 0.5
        theta_precision = 0.6 
        dist_precision = 3.0

    def odom_callback(self,data):
      x = data.pose.pose.orientation.x
      y = data.pose.pose.orientation.y
      z = data.pose.pose.orientation.z
      w = data.pose.pose.orientation.w
      self.pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]



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
            
    def distance_calculation (self , destination_x , destination_y) :

      theta_goal1 = np.arctan((destination_y - self.pose[1] / (destination_x - self.pose[0])))

      bot_theta1 = self.pose[2]

      position_error1 = np.sqrt(pow(destination_y - self.pose[1], 2) + pow(destination_x - self.pose[0], 2)) 

      theta_error1 = round(abs(bot_theta1 - theta_goal1) , 2)

      return [theta_error1 , position_error1]
      





    def image_showing (self):   

        theta_precision = 0.3 
        dist_precision = 600.0
        detect = detection()
        detection_result = detect.aruco_detection(self.cv1_image)  

        if detection_result[2] == 2 :

            ls = self.distance_calculation ( detection_result[1][0] , detection_result[1][1])
            theta_error = ls[0]
            position_error = ls[1]

            while  ( theta_error > theta_precision ) or (position_error >dist_precision ) :

              cv2.imshow("mask" , detection_result[0])
              rospy.loginfo("intial theta error" +str(theta_error))
              rospy.loginfo("intial position error" +str(position_error))

              if theta_error >theta_precision :


                

                ls = self.distance_calculation ( detection_result[1][0] , detection_result[1][1])
                theta_error = ls[0]
                position_error = ls[1]

                self.move ( 0.3 , -1*self.p*theta_error  )
                

                # cv2.imshow("theta_error" , detection_result[0])
                rospy.loginfo("theta_error+" +str(theta_error))

           
            # elif theta_error < theta_precision :


            #   self.move(0.5 , -self.p*theta_error - self.at)
            #   rospy.loginfo("theta_error- " +str(theta_error))
              elif position_error > dist_precision :
              
                ls = self.distance_calculation ( detection_result[1][0] , detection_result[1][1])
                theta_error = ls[0]
                position_error = ls[1]
                new_position_error = (position_error/100)*0.5
                self.move ( self.p*new_position_error , 0)
                rospy.loginfo("positon error >0" + str(new_position_error))
                # rospy.loginfo("theta_error >0" +str(theta_error))

                if theta_error > theta_precision:

                  ls = self.distance_calculation ( detection_result[1][0] , detection_result[1][1])
                  theta_error = ls[0]
                  position_error = ls[1]

                  self.move (0 , -1*self.p*theta_error )
                  rospy.loginfo("second_theta_error " +str(theta_error))
              else  :
                self.move(0,0)
                rospy.loginfo("stop the bot ")
            

        else :
          self.move(0,0.3)



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

    