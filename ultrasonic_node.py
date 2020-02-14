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
        self.ECHO_RIGHT = 16
        self.ECHO_LEFT = 26
        GPIO.setup(self.GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(self.ECHO_RIGHT, GPIO.IN)
        GPIO.setup(self.ECHO_LEFT, GPIO.IN)

    def distance(self, echo):
        # This function measures a distance
        MAX_TIME = 0.04  # max time waiting for response in case something is missed
        # Pulse the trigger line to initiate a measurement
        GPIO.output(self.GPIO_TRIGGER, True)
        time.sleep(0.00001)
        GPIO.output(self.GPIO_TRIGGER, False)
        # ensure start time is set in case of very quick return
        start = time.time()
        timeout = start + MAX_TIME

        # set line to input to check for start of echo response
        while GPIO.input(echo) == 0 and start <= timeout:
            start = time.time()

        stop = time.time()
        timeout = stop + MAX_TIME
        # Wait for end of echo response
        while GPIO.input(echo) == 1 and stop <= timeout:
            stop = time.time()

        GPIO.setup(self.GPIO_TRIGGER, GPIO.OUT)
        GPIO.output(self.GPIO_TRIGGER, False)

        elapsed = stop - start
        dista = (elapsed * 34300) / 2.0
        time.sleep(0.02)
        return dista

    def run_distance(self):
        right_dist1 = self.distance(self.ECHO_RIGHT)
        left_dist1 = self.distance(self.ECHO_LEFT)

        right_dist2 = self.distance(self.ECHO_RIGHT)
        left_dist2 = self.distance(self.ECHO_LEFT)

        difference_right = abs(right_dist1 - right_dist2)
        difference_left = abs(left_dist1 - left_dist2)

        ultra = TwoFloat()

        if difference_left < 20 and difference_right < 20:
            ultra.left_us = left_dist1
            ultra.right_us = right_dist1
            self.pub.publish(ultra)
            rospy.loginfo('{ULTRASUONI} Distanza sx: ' + str(left_dist1) + ", Distanza dx: " + str(right_dist1))
        else:
            pass


if __name__ == "__main__":
    ultra_suoni = UltraSuoni()
    rospy.init_node("ultrasuoni", anonymous=True)
    loop = rospy.Rate(ultra_suoni.node_rate)
    while not rospy.is_shutdown():
        ultra_suoni.run_distance()
        loop.sleep()
    GPIO.cleanup()
