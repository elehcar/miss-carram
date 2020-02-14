#!/usr/bin/env python
# coding=utf8
import rospy
import cv2
from landmark import Landmark
from std_msgs.msg import IntString, Float32
from sensor_msgs.msg import Image
import csv
from cv_bridge import CvBridge, CvBridgeError


class PlannerNode:

    def __init__(self):
        self.bridge_object = CvBridge()
        self.curve = False
        self.pub = rospy.Publisher("planning_topic", IntString, queue_size=10)
        rospy.Subscriber("qr_topic", Image, self.planner_callback, 0)
        rospy.Subscriber("magnetometer_topic", Float32, self.planner_callback, 1)
        self.landmark = Landmark()
        self.north = 0
        self.qr_code = None
        self.qr_distance = None
        self.lowlimit = None
        self.upperlimit = None

    def planner_callback(self, data, arg):
        raw_grades = 30  # verificare range
        msg = IntString()
        qr_distance_new = 0
        if arg == 1:
            self.north = data.data
        else:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, "bgr8")
            cropped = cv_image[0:250, 0:640]  # verificare taglio corretto dell'img
            self.qr_code, qr_distance_new = self.landmark.get_qrdata(cropped)
        if qr_distance_new == self.qr_distance:
            msg.distance = 200
        else:
            self.qr_distance = qr_distance_new
            msg.distance = qr_distance_new

        # msg.distance = self.qr_distance
        # calcolare gradi giusti!
        if self.qr_code == "1" or self.qr_code == "2":
            self.lowlimit = 280
            self.upperlimit = 350
            if self.qr_code == "2":
                self.curve = True
            else:
                self.curve = False
        elif self.qr_code == "3" or self.qr_code == "4":
            self.lowlimit = 125
            self.upperlimit = 180
            if self.qr_code == "4":
                self.curve = True
            else:
                self.curve = False
        elif self.qr_code == "5":
            self.lowlimit = 32
            self.upperlimit = 80
            self.curve = False
        else:  # landmark non visibile
            self.lowlimit = 0
            self.upperlimit = 359
            self.curve = False

        msg.curve = self.curve
        if self.lowlimit <= self.north <= self.upperlimit:
            msg.rotation = 0  # Nomral
        elif (self.lowlimit-raw_grades <= self.north < self.lowlimit) or (self.upperlimit < self.north <= self.upperlimit + raw_grades):
            msg.rotation = 1  # Turn Opposite
        else:  # completamente girato -> inversione
            msg.rotation = 2  # Invert

        print("{PLANNING_NODE} Rotation: " + str(msg.rotation) + ", Distance: " + str(msg.distance) + ",  Curva: " + str(msg.curve))
        self.pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("planning_node", anonymous=True)
    ob = PlannerNode()
    rospy.spin()
    # primo lato obliquo (landmark 1-2): 280(sx) 315(centro) 350(dx)
    # secondo lato obliquo (landmark 3-4): 125(sx) 150(centro) 180(dx)
    # lato dritto (landmark 5): 32(sx) 53(centro) 80(dx)

