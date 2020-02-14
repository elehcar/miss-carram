#!/usr/bin/env python
# coding=utf8

import rospy
import sys
from geometry_msgs.msg import Twist
from motor_driver import MotorDriver


class RobotMover(object):

    def __init__(self):
        self.previous_angular = 0
        self.previous_linear = 0.15
        rospy.Subscriber("cmd_vel_topic", Twist, self.cmd_vel_callback)
        self.motion_driver = MotorDriver()

    def cmd_vel_callback(self, data):
        linear_speed = data.linear.x
        angular_speed = data.angular.z
        if (linear_speed == -1000) and (angular_speed == -1000):
            self.motion_driver.change_speed(self.previous_linear, self.previous_angular)
            print("{ROBOT_MOVER} Linear: " + str(self.previous_linear) + ", Angular: " + str(self.previous_angular))
        else:
            self.previous_linear = linear_speed
            self.previous_angular = angular_speed
            self.motion_driver.change_speed(linear_speed, angular_speed)
            print("{ROBOT_MOVER} Linear: " + str(linear_speed) + ", Angular: " + str(angular_speed))


if __name__ == '__main__':
    rospy.init_node('vel_listener', anonymous=True)
    robot_mover = RobotMover()
    rospy.spin()

