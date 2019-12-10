#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def talker():
	pub = rospy.Publisher('speeds', Twist, queue_size= 10)
	rospy.init_node('sensehat', anonymous= True)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		msg = Twist()
		msg.linear.x= 
		msg.linear.y=
		msg.linear.z=
		msg.angular.x=
		msg.angular.y=
		msg.angular.z= 
		rospy.loginfo(msg)
		pub.publish(msg)
		rate.sleep()
if __name__== '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
