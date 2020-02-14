#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from sense_hat import SenseHat


class SenseNode:

	def __init__(self):
		self.node_rate = 10
		self.sense = SenseHat()
		self.pub = rospy.Publisher('magnetometer_topic', Float32, queue_size=10)

	def sensing(self):
		north = self.sense.get_compass()
		self.pub.publish(north)
		print("{SENSE_HAT} North: " + str(north))


if __name__ == '__main__':
	sense_hat = SenseNode()
	rospy.init_node("sense_hat", anonymous=True)
	loop = rospy.Rate(sense_hat.node_rate)
	while not rospy.is_shutdown():
		sense_hat.sensing()
		loop.sleep()

