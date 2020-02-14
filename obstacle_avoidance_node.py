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
        self.factor = 1
        self.node_rate = 10
        self.area = -1
        self.target_x = 0
        self.w = 640
        self.h = 480
        self.distanza_ostacolo = 0
        self.distanza_dx = 0
        self.distanza_sx = 0
        # This way we process only half the frames
        self.bridge_object = CvBridge()
        rospy.Subscriber("image_topic", Image, self.callback, 0)
        rospy.Subscriber("ultrasuoni_topic", TwoFloat, self.callback, 1)
        self.pub = rospy.Publisher("change_obstacle", Twist, queue_size= 1)
        self.distance = Distance()
        self.linear_vel_base = linear_vel_base
        self.angular_vel_base = angular_vel_base

    def callback(self, data, args):
        if args == 0:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, "bgr8")
            cropped = cv_image[100:480, 0:640]
            # cv2.imshow("im", cropped)
            # cv2.waitKey(0)

            self.area, self.target_x = self.distance.find_area(cropped)
            self.distanza_ostacolo = self.distance.distancetoCamera(self.area)
            if 70 < self.distanza_ostacolo <= 100:
                self.factor = 1
            elif 50 < self.distanza_ostacolo <= 70:
                self.factor = 1.2
            else:
                self.factor = 1.5
        else:
            self.distanza_sx = data.left_us
            self.distanza_dx = data.right_us

    def calc_speed(self):
        speed = Twist()
        if 100 < self.distanza_ostacolo <= 200:  # ostacolo non visibile o troppo lontano
            if self.distanza_sx < 15 and self.distanza_dx < 15:  # se ho ostacoli a dx e sx
                speed.linear.x = self.linear_vel_base
                speed.angular.z = 0
                print("{OBSTACLE_AVOIDANCE} Linear: " + str(speed.linear.x) + ", Angular: " + str(speed.angular.z))

            elif self.distanza_sx < 15:  # se ho ostacolo a sx vado a dx
                # imposto un'accelerazione angolare che lo fa spostare un po' verso dx
                speed.linear.x = self.linear_vel_base
                speed.angular.z = self.angular_vel_base
                print("{OBSTACLE_AVOIDANCE} Linear: " + str(speed.linear.x) + ", Angular: " + str(speed.angular.z))

            elif self.distanza_dx < 15:  # se ho ostacolo a dx vado a sx
                # imposto un'accelerazione angolare che lo fa spostare un po' verso sx
                speed.linear.x = self.linear_vel_base
                speed.angular.z = self.angular_vel_base * -1
                print("{OBSTACLE_AVOIDANCE} Linear: " + str(speed.linear.x) + ", Angular: " + str(speed.angular.z))

            else:
                speed.linear.x = -1000
                speed.angular.z = -1000
        else:  # ostacolo visibile
            if self.distanza_sx > 15 and self.distanza_dx > 15:  # non ho ostacoli vicini a dx e a sx -> considero l'ostacolo davanti
                if self.target_x < self.w / 2:
                    speed.linear.x = self.linear_vel_base
                    speed.angular.z = self.angular_vel_base * self.factor
                elif self.target_x > self.w / 2:
                    speed.linear.x = self.linear_vel_base
                    speed.angular.z = self.angular_vel_base * -1 * self.factor
                else:
                    speed.linear.z = self.linear_vel_base
                    speed.angular.z = self.angular_vel_base * -1 * self.factor
            elif self.distanza_dx < 15 and self.distanza_sx < 15:  # ho ostacoli in tutte le direzioni -> mi fermo
                speed.linear.x = 0
                speed.angular.z = 0
            elif self.distanza_sx < 15:  # ho ostacolo davanti e a sx -> vado a dx basandomi sul target x altrimenti dritto
                if self.target_x <= self.w / 2:
                    speed.linear.x = self.linear_vel_base
                    speed.angular.z = self.angular_vel_base * self.factor
                else:
                    speed.linear.x = self.linear_vel_base
                    speed.angular.z = 0
            else:  # ho ostacolo davanti e a dx -> vado a sx basandomi sul target x
                if self.target_x >= self.w/2:
                    speed.linear.x = self.linear_vel_base
                    speed.angular.z = self.angular_vel_base * -1 * self.factor
                else:
                    speed.linear.x = self.linear_vel_base
                    speed.angular.z = 0

            print("{OBSTACLE_AVOIDANCE + CAMERA} Linear: " + str(speed.linear.x) + ", Angular: " + str(speed.angular.z) + ", Distanza: " + str(self.distanza_ostacolo) + ", Target: " + str(self.target_x) + ", Area: " + str(self.area))
        self.pub.publish(speed)


if __name__ == '__main__':
    rospy.init_node('obstacle_avoidance', anonymous=True)
    obstacle_avoidance = ObstacleAvoidance(0.15, 0.26)
    loop = rospy.Rate(obstacle_avoidance.node_rate)
    while not rospy.is_shutdown():
        obstacle_avoidance.calc_speed()
        loop.sleep()
