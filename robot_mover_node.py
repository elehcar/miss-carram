#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Twist
from motor_driver import MotorDriver


class RobotMover(object):
    
    def __init__(self):
        rospy.Subscriber("cmd_vel_topic", Twist, self.cmd_vel_callback)
        self.motion_driver = MotorDriver()

    def cmd_vel_callback(self, data):
        linear_speed = data.linear.x
        angular_speed = data.angular.z
        rospy.loginfo(data.linear)
        self.motion_driver.change_speed(linear_speed, angular_speed)


if __name__ == '__main__':
    rospy.init_node('vel_listener', anonymous=True)
    robot_mover = RobotMover()
    rospy.spin()
