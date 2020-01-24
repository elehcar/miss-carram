#!/usr/bin/env python
import rospy
from dynamic_distance import Distance
from sensor_msgs.msg import Image
from std_msgs.msg import TwoFloat
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge, CvBridgeError


class ObstacleAvoidance(object):

    def __init__(self, linear_vel_base, angular_vel_base):
        self.target_x = 0
        self.target = 0
        self.w = 640
        self.h = 480
        self.distanza_ostacolo = 0
        self.distanza_dx = 0
        self.distanza_sx = 0
        # This way we process only half the frames
        self.bridge_object = CvBridge()
        rospy.Subscriber("image_topic", Image, self.image_callback)
        rospy.Subscriber("ultrasuoni_topic", TwoFloat, self.ultraSuoni_callback)
        self.pub = rospy.Publisher("change_obstacle", Twist, queue_size= 1)
        self.distance = Distance()
        self.linear_vel_base = linear_vel_base
        self.angular_vel_base = angular_vel_base

    def ultraSuoni_callback(self, data):
        self.distanza_sx = data.left_us
        self.distanza_dx = data.right_us

    def image_callback(self, data):
        # We select bgr8 because its the OpenCV encoding by default
        cv_image = self.bridge_object.imgmsg_to_cv2(data, "bgr8")
        area, self.target = self.distance.find_area(cv_image)
        self.distanza_ostacolo = self.distance.distancetoCamera(area)

    def calc_speed(self):
        speed = Twist()
        if self.distanza_ostacolo == 200:
            if self.distanza_sx < 5 and self.distanza_dx < 5:
                speed.linear.x = self.linear_vel_base
                speed.angular.z = 0
                self.pub.publish(speed)
            elif self.distanza_sx < 5:
                # imposto un'accelerazione angolare che lo fa spostare un po' verso dx
                speed.linear.x = self.linear_vel_base
                speed.angular.z = self.angular_vel_base * -1
                self.pub.publish(speed)
            elif self.distanza_dx < 5:
                # imposto un'accelerazione angolare che lo fa spostare un po' verso sx
                speed.linear.x = self.linear_vel_base
                speed.angular.z = self.angular_vel_base
                self.pub.publish(speed)
            else:
                pass
        elif self.distanza_sx > 5 and self.distanza_dx > 5:
            if self.target_x < self.w / 2:
                speed.linear.x = self.linear_vel_base
                speed.angular.z = self.angular_vel_base * -1
                self.pub.publish(speed)
            elif self.target_x > self.w /2:
                speed.linear.x = self.linear_vel_base
                speed.angular.z = self.angular_vel_base
                self.pub.publish(speed)
            else :
                speed.linear.z = self.linear_vel_base
                speed.angular.z = self.angular_vel_base
                self.pub.publish(speed)
        elif self.distanza_dx < 5 and self.distanza_sx < 5:
            speed.linear.x = 0
            speed.angular.z = 0
            self.pub.publish(speed)
        elif self.distanza_sx < 5:
            if self.target_x <= self.w / 2:
                speed.linear.x = self.linear_vel_base
                speed.angular.z = self.angular_vel_base* -1
                self.pub.publish(speed)
            else:
                speed.linear.x = self.linear_vel_base
                speed.angular.z = 0
                self.pub.publish(speed)
        else:
            if self.target_x >= self.w/2:
                speed.linear.x = self.linear_vel_base
                speed.angular.z = self.angular_vel_base
                self.pub.publish(speed)
            else:
                speed.linear.x = self.linear_vel_base
                speed.angular.z = 0
                self.pub.publish(speed)


if __name__ == '__main__':
    rospy.init_node('obstacle_avoidance', anonymous=True)
    obstacle_avoidance = ObstacleAvoidance(0.01, 0.1)
    while not rospy.is_shutdown():
        obstacle_avoidance.calc_speed()
        rospy.spin()

