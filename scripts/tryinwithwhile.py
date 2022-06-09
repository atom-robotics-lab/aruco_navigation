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

        self.velocity_msg = Twist()
        self.p = 0.01
        self.at = 0.001
        self.radius_threshold = 90
        self.id = None
        self.distance_precision = 30


    def move(self,linear,angular):
        self.velocity_msg.linear.x = linear
        self.velocity_msg.angular.z = angular 
        rospy.loginfo("linear velocity " +str(linear))
        rospy.loginfo("angular velocity " +str(angular) )
        self.pub.publish(self.velocity_msg)

    def callback(self,data):
        try:
            self.cv1_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            self.control_loop()
            
        except CvBridgeError as e :
            print(e)    

    def control_loop(self):

 
        detect = detection()
        self.Result = detect.aruco_detection(self.cv1_image)

        x_length = self.Result[0].shape[0]


    

        if (self.Result[3] != None ) and (self.Result[1] != None) and (self.Result[2] != None ):
            aruco_position =  self.Result[1][0]
            self.position_error = int(x_length)/2 - aruco_position


            
            rospy.loginfo("radius" +str(self.Result[2]))
            rospy.loginfo("position error " +str(self.position_error))
            
            if (self.Result[2] < self.radius_threshold) :

                if self.position_error > 0 :
                    self.move(0.3 , -self.at*self.position_error)
                    print("left")
                elif self.position_error < 0 :
                    self.move(0.3 , self.at*self.position_error)
                    print("right")
                else :
                    self.move(0.3 , 0)
                    print("straight")
            else :

                if self.position_error > 0 :
                    self.move(0 , -self.at*self.position_error)
                elif self.position_error < 0 :
                    self.move(0 , self.at*self.position_error)
                else :
                    self.move(0 , 0)
        else   :
            # self.move(0, 0.2)
            pass
        cv2.imshow("image" , self.Result[0])
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

                



