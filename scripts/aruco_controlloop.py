#! /usr/bin/env python3
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from opencv11 import detection
# from anushkacv import aruco_detection
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

        self.velocity_msg = Twist()
        self.p = 0.01
        self.at = 0.001
        self.radius_threshold = 150
        self.id = None
        self.distance_precision = 30
        self.theta_precision = 0.5
        self.pi = 1.5735

    def odom_callback(self,data): 

        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w
        self.pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]


    def move(self,linear,angular):
        self.velocity_msg.linear.x = linear
        self.velocity_msg.angular.z = angular 
        rospy.loginfo("linear velocity " +str(linear))
        rospy.loginfo("angular velocity " +str(angular) )
        self.pub.publish(self.velocity_msg)


    def parking_bot(self):

        bot_theta = self.pose[2]
        theta_goal = np.arcsin(1)

        rospy.loginfo("bot theta " +str(bot_theta))
        self.move(0,0.5)
        # rospy.loginfo("sin value " +str(np.arcsin(1)) )
        # bot_reached = False
        # while bot_reached == False :

        #     self.move(0,0.3)

        #     # new_theta =  - abs(self.pose[2]) + abs(bot_theta)
        #     # rospy.loginfo("bot theta " +str(self.pose[2]))
        #     # rospy.loginfo("new_theta" +str(new_theta))
        #     if 1.57 > abs(new_theta) :
        #         self.move(0,0.1)
        #     else :
        #         self.move(0,0)
        #         print("parked")
        #         bot_reached = True
                

        
            
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


    

        if (self.Result[3] == 2 ) and (self.Result[1] != None) and (self.Result[2] != None ):
            aruco_position =  self.Result[1][0]
            self.theta_error = int(x_length)/2 - aruco_position



            rospy.loginfo("aruco_postion" +str(aruco_position))
            rospy.loginfo("x lenght" +str(x_length))
            rospy.loginfo("radius" +str(self.Result[2]))
            rospy.loginfo("theta error " +str(self.theta_error))
            
            if (self.Result[2] < self.radius_threshold) :

                if self.theta_error > 0  and (self.theta_precision < abs(self.theta_error)) :
                    self.move(0.3 , self.at*self.theta_error)
                    print("left")
                elif self.theta_error < 0 and (self.theta_precision < abs(self.theta_error)) :
                    self.move(0.3 , self.at*self.theta_error)
                    print("right")
                else :
                    self.move(0.3 , 0)
                    print("straight")
            else :

                if self.theta_error > 0 and (self.theta_precision > abs(self.theta_error))  :
                    self.move(0 , self.at*self.theta_error)
                elif self.theta_error < 0 and (self.theta_precision > abs(self.theta_error) ):
                    self.move(0 , self.at*self.theta_error)
                else :
                    self.move(0 , 0)
                    self.parking_bot()
        else   :
            self.move(0, 0.2)
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

                



