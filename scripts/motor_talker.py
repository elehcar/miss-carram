#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

def talker():
    pub = rospy.Publisher('wheel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = Twist()
        msg.linear.x = 1.0
        msg.linear.y = 1.2
        msg.linear.z = 1.9
        msg.angular.x = 1
        msg.angular.y = 1
        msg. angular.z = 2.1
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
