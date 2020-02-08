#!/usr/bin/env python
# coding=utf8
import rospy
import cv2
from landmark import Landmark
from std_msgs.msg import IntString
from sensor_msgs.msg import Image


class PlannerNode:

    def __init__(self):
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
            msg.distance = 0
        else:
            self.qr_distance = qr_distance_new
            msg.distance = qr_distance_new

        msg.distance = self.qr_distance
        # calcolare gradi giusti!
        if self.qr_code == "landmark1":
            self.lowlimit = 200
            self.upperlimit = 300
        elif self.qr_code == "landmark2":
            self.lowlimit = 100
            self.upperlimit = 200
        elif self.qr_code == "landmark3":
            self.lowlimit = 0
            self.upperlimit = 100
        else:  # landmark non visibile
            self.lowlimit = 0
            self.upperlimit = 359

        if lowlimit <= north <= upperlimit:
            msg.rotation = "Normal"
        elif (lowlimit-raw_grades <= north < lowlimit) or (upperlimit < north <= upperlimit + raw_grades):
            msg.rotation = "Turn_Opposite"
        else:  # completamente girato -> inversione
            msg.rotation = "Invert"
        self.pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("planning_node", anonymous=True)
    ob = PlannerNode()
    rospy.spin()
