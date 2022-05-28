#! /usr/bin/env python3

# Import all the nessecary packages
import rospy
from geometry_msgs.msg import Twist
from aruco_image_processing import ArucoMarker
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image


# Create a class for object follower
class ArucoController:
  def __init__(self):
    self.bridge = CvBridge() # Creating an Instance of CV Bridge
    self.image_sub =rospy.Subscriber("/rrbot/camera1/image_raw",Image,self.callback) # Subsciber for the Image feed
    self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)                     # Publisher to publish the velocities
    self.velocity_msg = Twist()  # Creating a messgae from the Twist template  
    
    # Setting the non required velocities to zero
    self.velocity_msg.linear.y = 0
    self.velocity_msg.linear.z = 0
    self.velocity_msg.angular.x = 0
    self.velocity_msg.angular.y = 0

    self.radius_threshold = 130
    self.pl =  0.015                         
    self.pa =  0.003
    self.ia =  0.000005
    self.abuffer = 2
    self.lbuffer = 1

    self.sum_ae= 0     # Sum of the error
    
  def callback(self,data):
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")   # Converting Image to CV2 comatible datatype
      self.control_loop()      
      
    except CvBridgeError as e:
      print(e)
      self.move(0,0)
      
  def control_loop(self):
    sc = ArucoMarker() # Creating an object of the class Image processing to process the incoming image
    result=sc.find_aruco(self.cv_image) # process Image to detect object in the image
    x_length=result[0].shape[0]
    x =int(x_length/2)    # Center of the image
    
    '''
    if(result[3]== None and result[2]==None): # No object found
      self.move(0,0)
      self.at = "Finding Object"
      self.lt = "Stop"
    else:  # Object had been detected in the Image feed
      x_pos=result[2][0]
      ae=x-x_pos        # Calculating the Error
      self.sum_ae+=ae   # Adding the error
      if(result[3]<self.radius_threshold-self.lbuffer): # Object is farther than the threshold
        if result[2][0]>(x_length/2+self.abuffer): # Object is Towards Right
          self.move(self.pl*(self.radius_threshold-result[3]),self.pa*ae + self.ia*self.sum_ae)          
          self.at = "Right==>"
          self.lt = "Go Forward"

        elif result[2][0]<(x_length/2-self.abuffer): # Object is Towards Left
          self.move(self.pl*(self.radius_threshold-result[3]),self.pa*ae + self.ia*self.sum_ae)          
          self.at = "<==Left"
          self.lt = "Go Forward"

        else:   # Object is in Center
          self.move(self.pl*(self.radius_threshold-result[3]),0)          
          self.at = "Center"
          self.lt = "Go Forward"
      
      elif(result[3]>self.radius_threshold+self.lbuffer): # Object is Nearer than the threshold
        if result[2][0]>(x_length/2+self.abuffer): # Object is Towards Right
          self.move(self.pl*(self.radius_threshold-result[3]),self.pa*ae + self.ia*self.sum_ae)
          self.at = "Right==>"
          self.lt = "Go Backward"

        elif result[2][0]<(x_length/2-self.abuffer): # Object is Towards Left         
          self.move(self.pl*(self.radius_threshold-result[3]),self.pa*ae + self.ia*self.sum_ae) 
          self.at = "<==Left"
          self.lt = "Go Backward"

        else:  # Object is in Center
          self.move(self.pl*(self.radius_threshold-result[3]),0)
          self.at = "Center"
          self.lt = "Go Backward"

      else:  # Object is at the correct Distance
        if result[2][0]>(x_length/2+self.abuffer): # Object is Towards Right    
          self.move(0,self.pa*ae+self.ia*self.sum_ae)
          self.at = "Right==>"
          self.lt = "Stop"


        elif result[2][0]<(x_length/2-self.abuffer): # Object is Towards Left       
          self.move(0,self.pa*ae+self.ia*self.sum_ae) 
          self.at = "<==Left"
          self.lt = "Stop"

        else: # Object is in Center         
          self.move(0,0)
          self.at = "Center"
          self.lt = "Stop"

      cv2.putText(result[1],"Area = "+str(round(3.14*result[3]*result[3],2)),(x-140,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2,cv2.LINE_AA)'''

    # Adding the Text to the frame
    #cv2.putText(result[0],self.at,(x-60,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2,cv2.LINE_AA)
    #cv2.putText(result[0],self.lt,(x-70,750),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2,cv2.LINE_AA)
        
    #Showing the Original Frame and the Masked Frame
    cv2.imshow("Frame",result[0])
    #mask3=cv2.cvtColor(result[1],cv2.COLOR_GRAY2BGR)
    #im_thresh_color = cv2.bitwise_and(result[0],mask3)
    #cv2.line(im_thresh_color , (x,0),(x,800),(255,0,0),2)
    #cv2.imshow("Mask",im_thresh_color)
    
    #cv2.imshow("Mask",im_thresh_color)
    cv2.waitKey(1)
    

  # Creating a function move that will publish the required velocities
  def move(self, linear, angular):
    linear = min(linear,2) # Creating a maximum limit for the linear velocity
    self.velocity_msg.linear.x = linear
    self.velocity_msg.angular.z = angular
    self.pub.publish(self.velocity_msg)  # Publish the velocities to the topic

def main():
  rospy.init_node("aruco_controller",anonymous=True) # Initialising the node Obj_follower
  ac = ArucoController()   # Create an object of the ObjectFollower Class
  try:
    rospy.spin()
    
  except:
    print("error")


  cv2.destroyAllWindows()


main()

	
