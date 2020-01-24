#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
import time
from time import sleep
from std_msgs.msg import TwoFloat


class UltraSuoni(object):

    def __init__(self):
        self.node_rate = 10
        self.pub = rospy.Publisher("ultrasuoni_topic", TwoFloat, queue_size=1)
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.GPIO_TRIGGER = 4
        self.ECHO_RIGHT = 26
        self.ECHO_LEFT = 16
        GPIO.setup(self.GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(self.ECHO_RIGHT, GPIO.IN)
        GPIO.setup(self.ECHO_LEFT, GPIO.IN)

    def distance(self, e):
        GPIO.output(self.GPIO_TRIGGER, False)
        time.sleep(0.1)
        GPIO.output(self.GPIO_TRIGGER, True)
        time.sleep(0.00001)
        GPIO.output(self.GPIO_TRIGGER, False)

        while GPIO.input(e) == 0:
            start_time = time.time()

        while GPIO.input(e) == 1:
            end_time = time.time()

        time_elapsed = end_time - start_time
        dist = (time_elapsed * 34300) / 2
        return dist

    def run_distance(self):
        right_dist = self.distance(self.ECHO_RIGHT)
        left_dist = self.distance(self.ECHOS_LEFT)

        ultra = TwoFloat()
        ultra.left_us = left_dist
        ultra.right_us = right_dist
        self.pub.publish(ultra)
        rospy.loginfo('Distanza sinistra: ' + str(left_dist) + ", Distanza destra: " + str(right_dist))


if __name__ == "__main__":
    ultra_suoni = UltraSuoni()
    rospy.init_node("ultrasuoni", anonymous=True)
    loop = rospy.Rate(ultra_suoni.node_rate)
    while not rospy.is_shutdown():
        ultra_suoni.run_distance()
        loop.sleep()
    GPIO.cleanup()
