#!/usr/bin/env python
# coding=utf8
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist


class DecisionNode(object):

    def __init__(self):
        self.ob_linear = None
        self.line_linear = None
        self.ob_angular = None
        self.line_angular = None
        rospy.Subscriber("line_foll_topic", Twist, self.line_callback, 0)
        rospy.Subscriber("change_obstacle", Twist, self.obstacle_callback, 1)
        self.pub = rospy.Publisher("cmd_vel_topic", Twist, queue_size=10)

    def line_callback(self, data):
        self.line_linear = data.linear.x
        self.line_angular = data.angular.z

    def obstacle_callback(self, data):
        self.ob_linear = data.linear.x
        self.ob_angular = data.angular.z

    def listener(self):
        speed = Twist()
        if (self.ob_linear is None) and (self.ob_angular is None):
            speed.linear.x = self.line_linear
            speed.angular.z = self.line_angular
        else:
            speed.linear.x = self.ob_linear
            spped.angular.z = self.ob_angular
        self.pub.publish(speed)


if __name__ == "__main__":
    rospy.init_node("decision_node", anonymous=True)
    while not rospy.is_shutdown():
        listener()
        rospy.spin()




