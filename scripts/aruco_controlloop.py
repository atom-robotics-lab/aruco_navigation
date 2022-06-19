#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from opencv import detection
from cv_bridge import CvBridge, CvBridgeError

import cv2
from sensor_msgs.msg import Image


class Robot_Controller:
    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw", Image, self.callback)
        self.image_pub = rospy.Publisher("image_topic_2", Image)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.velocity_msg = Twist()
        self.angular_p = rospy.get_param("aruco_navigation/angular_p")
        self.radius_threshold = rospy.get_param("aruco_navigation/radius_threshold")
        self.theta_precision = rospy.get_param("aruco_navigation/theta_precision")
        self.linear_p = rospy.get_param("aruco_navigation/linear_p")

        self.id = None
        self.buffer = 0.001
        self.detect = detection()
        self.detect.T = 3
        self.lt = ""
        self.at = ""

    def move(self, linear, angular):
        self.velocity_msg.linear.x = linear
        self.velocity_msg.angular.z = angular
        rospy.loginfo("linear velocity " + str(linear))
        rospy.loginfo("angular velocity " + str(angular))

        self.pub.publish(self.velocity_msg)

    def callback(self, data):
        try:
            self.cv1_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.control_loop()
        except CvBridgeError as e:
            print(e)

    def direction(self, markerID):

        if markerID == 1:
            return 0.3, "Turning Left ", "ID - 1"
        elif markerID == 2:
            return -0.3, "Turning Right", "ID -2"
        else:
            return 0, " Parked ", "ID - 3"

    def control_loop(self):

        self.Result = self.detect.aruco_detection(self.cv1_image)

        x_length = self.Result[0].shape[0]
        x_length = x_length + 40

        if self.detect.markerID1 != 0 and self.detect.center != None and self.detect.radius1 != None:
            aruco_position = self.Result[1][0]
            self.theta_error = int(x_length) / 2 - aruco_position

            rospy.loginfo("aruco_postion" + str(aruco_position))
            rospy.loginfo("x lenght" + str(x_length))
            rospy.loginfo("radius" + str(self.Result[2]))
            rospy.loginfo("theta error " + str(self.theta_error))

            if (self.Result[2] < self.radius_threshold):
                self.detect.T = 3
                if self.theta_error > 0 and (self.theta_precision < abs(self.theta_error)):
                    self.move(self.linear_p * (self.radius_threshold - self.Result[2]),
                              self.angular_p * self.theta_error)

                    self.at = " <- Left"
                    self.lt = ""
                    print("left")

                elif self.theta_error < 0 and (self.theta_precision < abs(self.theta_error)):
                    self.move(self.linear_p * (self.radius_threshold - self.Result[2]),
                              self.angular_p * self.theta_error - self.buffer * self.theta_error)
                    self.at = "  Right->"
                    self.lt = ""
                    print("right")

                else:
                    self.move(self.linear_p * (self.radius_threshold - self.Result[2]), 0)
                    self.at = " centre "
                    self.lt = ""

                    print("straight")
            else:

                if self.theta_error > 0 and (self.theta_precision > abs(self.theta_error)):
                    self.move(0, self.angular_p * self.theta_error)

                elif self.theta_error < 0 and (self.theta_precision > abs(self.theta_error)):
                    self.move(0, self.angular_p * self.theta_error)
                else:
                    angular = self.direction(self.detect.markerID1)
                    self.lt = angular[1]
                    self.at = ""
                    self.move(0, angular[0])
        else:
            self.move(0,0.2)
            self.at = "Finding Aruco"
            
        cv2.putText(self.Result[4], self.lt, (300, 800), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(self.Result[4], self.at, (350, 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 3, cv2.LINE_AA)

        cv2.imshow("image", self.Result[4])
        cv2.waitKey(1)


def main():
    rospy.init_node("robot controller", anonymous=True)
    of = Robot_Controller()
    try:
        rospy.spin()

    except:

        print("error")
    cv2.destroyAllWindows()


main()


