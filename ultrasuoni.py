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
        GPIO.setup(self.GPIO_TRIGGER, GPIO.OUT)

        ECHOS = {"ECHO_LEFT": 26, "ECHO_RIGHT": 16}
        for e in ECHOS:
            GPIO.setup(ECHOS[e], GPIO.IN)

    def distance(self, e):
        new_reading = False
        counter = 0
        GPIO.output(self.GPIO_TRIGGER, True)
        time.sleep(0.00001)
        GPIO.output(self.GPIO_TRIGGER, False)

        while GPIO.input(e) == 0:
            pass
            counter += 1
            if counter == 5000:
                new_reading = True
                break
        start_time = time.time()

        if new_reading:
            return False

        while GPIO.input(e) == 1:
            pass
        end_time = time.time()

        time_elapsed = end_time - start_time
        dist = (time_elapsed * 34300) / 2
        return dist

    def run_distance(self):
        sensors = []
        for e in self.ECHOS:
            dist = self.distance(self.ECHOS[e])
            sensors.append("{}: {}".format(e, dist))
            time.sleep(1)

        ultra = TwoFloat()
        ultra.left_us = sensors.dist[0][0]
        ultra.right_us = sensors.dist[1][0]
        try:
            self.pub.publish(ultra)
            rospy.loginfo('publishing ultrasonic distances')
        except ROSSerializationException():
            rospy.loginfo('invalid ultrasonic distances')

if __name__ == "__main__":
    ultra_suoni = UltraSuoni()
    rospy.init("ultrasuoni", anonymous=True)
    loop = rospy.Rate(ultra_suoni.node_rate)
    while not rospy.is_shutdown():
        ultra_suoni.run_distance()
        loop.sleep()
