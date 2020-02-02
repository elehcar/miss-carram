#!/usr/bin/env python
# coding=utf8
import rospy
import cv2
import numpy as np
from std_msgs.msg import TwoBool, Int16
from geometry_msgs.msg import Twist

# sulla base dei valori restituiti dai sensori, calcola accelerazione e velocità angolare coi seguenti criteri:
# se entrambi sono false va dritto perché significa che si trova in una delle due carreggiate
# quando uno dei due è true, va in quella direzione (ES se è true il sensore destro significa che deve girarsi
# un po' a destra)


class LineFollower(object):

    def __init__(self, linear_vel_base, angular_vel_base):
        self.counter = 0
        self.ir_sub = rospy.Subscriber("ir_node_topic", TwoBool, self.line_foll_callback)
        self.line_planning_pub = rospy.Publisher("count_round", Int16, queue_size=1)
        self.line_foll_pub = rospy.Publisher("line_foll_topic", Twist, queue_size=1)
        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.linear_vel_base = linear_vel_base
        self.angular_vel_base = angular_vel_base

    def line_foll_callback(self, ir_sensors):
        left_ir = ir_sensors.left_ir
        right_ir = ir_sensors.right_ir

        if left_ir and right_ir:
            # è arrivato alla linea di inizio/fine giro
            self.counter = self.counter + 1
            self.line_planning_pub.publish(counter)
            self.cmd_vel.angular.z = -1000
            self.cmd_vel.linear.x = -1000
        elif left_ir and not right_ir:
            # il sensore sx rileva la linea -> vado a sinistra
            self.cmd_vel.angular.z = self.angular_vel_base * -1
            self.cmd_vel.linear.x = self.linear_vel_base
            print("{LINE_FOLLOWER} SPEED==>[" + str(self.cmd_vel.linear.x) + "," + str(self.cmd_vel.angular.z) + "]")

        elif right_ir and not left_ir:
            # il sensore dx rileva la linea -> vado a destra
            self.cmd_vel.angular.z = self.angular_vel_base
            self.cmd_vel.linear.x = self.linear_vel_base
            print("{LINE_FOLLOWER} SPEED==>[" + str(self.cmd_vel.linear.x) + "," + str(self.cmd_vel.angular.z) + "]")

        else:
            # sono nella carreggiata -> pubblico non fare nulla
            self.cmd_vel.angular.z = -1000
            self.cmd_vel.linear.x = -1000

        self.line_foll_pub.publish(self.cmd_vel)


if __name__ == "__main__":
    rospy.init_node("line_follower", anonymous=True)
    line_follower = LineFollower(0.05, 0.1)
    rospy.spin()

