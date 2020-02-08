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
        self.rotation = "Normal"
        self.qr_distance = 0
        self.qr_distance_new = -1
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
            print("{DECISION_NODE} SPEED==> Line: [" + str(speed.linear.x) + "," + str(speed.angular.z) + "]")

        else:
            if self.ob_angular < 0:  # sinistra
                self.orientation = 0
            else:  # destra
                self.orientation = 1
            self.linear = self.ob_linear
            self.angular = self.ob_angular

        if self.qr_distance_new == 0:  # la camera non vede landmark
            if self.qr_distance > 30:  # non vedo landamark ma ancora ero lontano -> significa che non l'ha superato
                if self.rotation == "Inverse":
                    self.linear = 0
                    self.angular = self.ob_angular * 2
                else:  # self.rotation == "Turn_Opposite"
                    self.linear = self.linear * -1
            # non vedo il landmark ma l'ultima volta che l'ho visto ero molto vicino -> l'ho superato e sono in curva
            # in questo caso mi baso sui valori precedenti
        else:  # vedo un landmark -> aggiorno la distanza
            self.qr_distance = self.qr_distance_new

        speed.linear.x = self.linear
        speed.angular.z = self.angular
        print("{DECISION_NODE} SPEED==> Obstacle: [" + str(speed.linear.x) + "," + str(speed.angular.z) + "]")

        self.pub.publish(speed)


if __name__ == "__main__":
    rospy.init_node("decision_node", anonymous=True)
    ob = DecisionNode()
    rospy.spin()
