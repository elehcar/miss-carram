#!/usr/bin/env python
# coding=utf8
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist


class DecisionNode(object):

    def __init__(self):
        self.orientation = None
        self.ob_linear = -1000
        self.line_linear = -1000
        self.ob_angular = -1000
        self.line_angular = -1000
        rospy.Subscriber("line_foll_topic", Twist, self.decision_callback, 0)
        rospy.Subscriber("change_obstacle", Twist, self.decision_callback, 1)
        self.pub = rospy.Publisher("cmd_vel_topic", Twist, queue_size=1)

    def decision_callback(self, data, args):
        speed = Twist()

        if args == 0:
            self.line_linear = data.linear.x
            self.line_angular = data.angular.z
        else:
            self.ob_linear = data.linear.x
            self.ob_angular = data.angular.z

        if (self.ob_linear == -1000) and (self.ob_angular == -1000):
            if (self.orientation == 0 and self.line_angular < 0) or (self.orientation == 1 and self.line_angular > 0):
                speed.angular.z = self.line_angular * -1
            else:
                speed.angular.z = self.line_angular
            self.orientation = None
            speed.linear.x = self.line_linear
            print("{DECISION_NODE} SPEED==> Line: [" + str(speed.linear.x) + "," + str(speed.angular.z) + "]")

        else:
            if self.ob_angular < 0:  # sinistra
                self.orientation = 0
            else:  # destra
                self.orientation = 1
            speed.linear.x = self.ob_linear
            speed.angular.z = self.ob_angular
            print("{DECISION_NODE} SPEED==> Obstacle: [" + str(speed.linear.x) + "," + str(speed.angular.z) + "]")

        self.pub.publish(speed)


if __name__ == "__main__":
    rospy.init_node("decision_node", anonymous=True)
    ob = DecisionNode()
    rospy.spin()
