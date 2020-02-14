#!/usr/bin/env python
# coding=utf8
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import IntString


class DecisionNode(object):

    def __init__(self):
        self.orientation = None
        self.rotation = 0
        self.curve = False
        self.qr_distance = 0
        self.qr_distance_new = 0
        self.ob_linear = -1000
        self.line_linear = -1000
        self.ob_angular = -1000
        self.line_angular = -1000
        self.linear = -1000
        self.angular = -1000
        rospy.Subscriber("line_foll_topic", Twist, self.decision_callback, 0)
        rospy.Subscriber("change_obstacle", Twist, self.decision_callback, 1)
        rospy.Subscriber("planning_topic", IntString, self.decision_callback, 2)
        self.pub = rospy.Publisher("cmd_vel_topic", Twist, queue_size=1)

    def decision_callback(self, data, args):
        speed = Twist()
        if args == 0:
            self.line_linear = data.linear.x
            self.line_angular = data.angular.z
        elif args == 2:
            self.rotation = data.rotation
            self.qr_distance_new = data.distance
            self.curve = data.curve
        else:
            self.ob_linear = data.linear.x
            self.ob_angular = data.angular.z

        if (self.ob_linear == -1000) and (self.ob_angular == -1000):
            if (self.orientation == 0 and self.line_angular < 0) or (self.orientation == 1 and self.line_angular > 0):
                self.angular = self.line_angular * -1
            else:
                self.angular = self.line_angular
            self.orientation = None
            self.linear = self.line_linear
            print("{DECISION_NODE} SPEED==> Line: [" + str(self.linear) + "," + str(self.angular) + "]")

        else:
            if self.ob_angular < 0:  # sinistra
                self.orientation = 0
            else:  # destra
                self.orientation = 1
            self.linear = self.ob_linear
            self.angular = self.ob_angular
            print("{DECISION_NODE} SPEED==> Obstacle: [" + str(self.linear) + "," + str(self.angular) + "]")


        # la camera non vede landmark -> casi in cui supponiamo di essere contro il muro
        if (self.qr_distance_new == 200 and not self.curve) or (self.qr_distance_new == 200 and self.curve and self.qr_distance > 30):
            if self.rotation == 2:
                self.linear = 0
                self.angular = self.ob_angular * 2
            elif self.rotation == 1:  # self.rotation == "Turn_Opposite"
               print("TURN OPPOSITE:")
               self.linear = self.linear * -1
        elif self.rotation == 0 or self.qr_distance_new < 200:
            self.qr_distance = self.qr_distance_new

        speed.linear.x = self.linear
        speed.angular.z = self.angular

        self.pub.publish(speed)
        print("{PUBLISHING: }  Linear: " + str(speed.linear.x) + ", Angular: " + str(speed.angular.z))


if __name__ == "__main__":
    rospy.init_node("decision_node", anonymous=True)
    ob = DecisionNode()
    rospy.spin()

