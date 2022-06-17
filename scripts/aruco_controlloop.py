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
        self.theta_precision =rospy.get_param("aruco_navigation/theta_precision") 
        self.linear_p=rospy.get_param("aruco_navigation/linear_p") 
       
        self.id = None
        
        self.detect = detection()
        self.detect.T = 3

 

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
            rospy.loginfo("Marker Id = " + str(self.detect.markerID1))
            rospy.loginfo("radius = " + str(self.detect.radius1))

        except CvBridgeError as e:
            print(e)

    def control_loop(self):

        self.Result = self.detect.aruco_detection(self.cv1_image)

        x_length = self.Result[0].shape[0]

        flag = True
        if self.detect.markerID1 == 1 and self.detect.center != None and self.detect.radius1 != None:
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

                    self.detect.T = 1
                    cv2.putText(self.Result[0], "STRAIGHT",
                                (0, 200),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                2, (600, 600, 600), 2)

                    print("left")
                elif self.theta_error < 0 and (self.theta_precision < abs(self.theta_error)):
                    self.move(self.linear_p * (self.radius_threshold - self.Result[2]),
                              self.angular_p * self.theta_error)
                    self.detect.T = 2
                    cv2.putText(self.Result[0], "STRAIGHT",
                                (0, 200),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                2, (600, 600, 600), 2)

                    print("right")
                else:
                    self.move(self.linear_p * (self.radius_threshold - self.Result[2]), 0)
                    self.detect.T = 3
                    cv2.putText(self.Result[0], "STRAIGHT",
                                (0, 200),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                2, (600, 600, 600), 2)

                    print("straight")
            else:

                if self.theta_error > 0 and (self.theta_precision > abs(self.theta_error)):
                    self.move(0, self.angular_p * self.theta_error)
                elif self.theta_error < 0 and (self.theta_precision > abs(self.theta_error)):
                    self.move(0, self.angular_p * self.theta_error)
                else:
                    self.detect.T == 1
                    cv2.putText(self.Result[0], "LEFT",
                                (0, 200),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                2, (600, 600, 600), 2)

                    self.move(0, 0.2)

        elif self.detect.markerID1 == 2 and self.detect.radius1 != None and self.detect.center != None:
            self.detect.T = 3
            Flag = False
            aruco_position = self.Result[1][0]
            self.theta_error = int(x_length) / 2 - aruco_position

            rospy.loginfo("aruco_postion" + str(aruco_position))
            rospy.loginfo("x lenght" + str(x_length))
            rospy.loginfo("radius" + str(self.Result[2]))
            rospy.loginfo("theta error " + str(self.theta_error))

            if (self.Result[2] < self.radius_threshold):

                if self.theta_error > 0 and (self.theta_precision < abs(self.theta_error)):
                    self.move(self.linear_p * (self.radius_threshold - self.Result[2]), self.angular_p * self.theta_error)
                    self.detect.T == 1
                    cv2.putText(self.Result[0], "STRAIGHT",
                                (0, 200),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                2, (600, 600, 600), 2)

                    print("left")
                elif self.theta_error < 0 and (self.theta_precision < abs(self.theta_error)):
                    self.move(self.linear_p * (self.radius_threshold - self.Result[2]), self.angular_p * self.theta_error)
                    self.detect.T == 2
                    cv2.putText(self.Result[0], "STRAIGHT",
                                (0, 200),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                2, (600, 600, 600), 2)

                    print("right")
                else:
                    self.move(self.linear_p * (self.radius_threshold - self.Result[2]), 0)
                    self.detect.T == 3
                    cv2.putText(self.Result[0], "STRAIGHT",
                                (0, 200),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                2, (600, 600, 600), 2)

                    print("straight")
            else:

                if self.theta_error > 0 and (self.theta_precision > abs(self.theta_error)):
                    self.move(0, self.angular_p * self.theta_error)
                elif self.theta_error < 0 and (self.theta_precision > abs(self.theta_error)):
                    self.move(0, self.angular_p * self.theta_error)
                else:
                    self.detect.T == 2
                    cv2.putText(self.Result[0], "RIGHT",
                                (0, 200),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                2, (600, 600, 600), 2)

                    self.move(0, -0.2)

        elif self.detect.markerID1 == 3 and self.detect.radius1 != None and self.detect.center != None:  # yaha 70 ki jaga value daliyo jo condition satisfy kar jaye and bot starting mai 3 ki taraf na jaye
            self.detect.T = 3
            aruco_position = self.Result[1][0]
            self.theta_error = int(x_length) / 2 - aruco_position

            rospy.loginfo("aruco_postion" + str(aruco_position))
            rospy.loginfo("x lenght" + str(x_length))
            rospy.loginfo("radius" + str(self.Result[2]))
            rospy.loginfo("theta error " + str(self.theta_error))

            if (self.Result[2] < self.radius_threshold):
                if self.theta_error > 0 and (self.theta_precision < abs(self.theta_error)):
                    self.move(self.linear_p * (self.radius_threshold - self.Result[2]), self.angular_p * self.theta_error)
                    cv2.putText(self.Result[0], "STRAIGHT",
                                (0, 200),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                2, (600, 600, 600), 2)

                    print("left")
                elif self.theta_error < 0 and (self.theta_precision < abs(self.theta_error)):
                    self.move(self.linear_p * (self.radius_threshold - self.Result[2]), self.angular_p * self.theta_error)
                    cv2.putText(self.Result[0], "STRAIGHT",
                                (0, 200),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                2, (600, 600, 600), 2)

                    print("right")
                else:
                    self.move(self.linear_p * (self.radius_threshold - self.Result[2]), 0)
                    self.detect.T == 3
                    cv2.putText(self.Result[0], "RIGHT",
                                (0, 200),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                2, (600, 600, 600), 2)

                    print("straight")
            else:

                if self.theta_error > 0 and (self.theta_precision > abs(self.theta_error)):
                    self.move(0, self.angular_p * self.theta_error)
                elif self.theta_error < 0 and (self.theta_precision > abs(self.theta_error)):
                    self.move(0, self.angular_p * self.theta_error)
                else:
                    cv2.putText(self.Result[0], "REACHED",
                                (0, 200),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                2, (600, 600, 600), 2)
                    self.move(0, 0)

        elif self.detect.markerID1 > 3:
            self.detect.T == 3
            cv2.putText(self.Result[0], "RIGHT",
                        (0, 200),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        2, (600, 600, 600), 2)
            pass



        else:
            if flag == True:
                self.detect.T = 1
                cv2.putText(self.Result[0], "LEFT",
                            (0, 200),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            2, (600, 600, 600), 2)

                self.move(0, 0.2)
                pass
            elif flag == False:
                self.detect.T = 2
                cv2.putText(self.Result[0], "RIGHT",
                            (0, 200),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            2, (600, 600, 600), 2)

                self.move(0, -0.2)
                pass
            else:
                pass

        cv2.imshow("image", self.Result[0])
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