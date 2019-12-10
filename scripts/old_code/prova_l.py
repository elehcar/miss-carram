#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(stringa):
    rospy.loginfo(rospy.get_caller_id() + "String recived: %s", stringa.data)

def distance_getter():
    rospy.init_node('distance_get', anonymous=True)
    rospy.Subscriber('distance', String, callback)

    rospy.spin()

if __name__ == '__main__':
    distance_getter()
