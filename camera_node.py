#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import time
from picamera import PiCamera
from picamera.array import PiRGBArray


class ImagePublisher(object):

    def __init__(self):
        self.node_rate = 1
        self.image_pub = rospy.Publisher("image_topic", Image, queue_size=1)
        self.bridge = CvBridge()
        self.camera = PiCamera()
        self.camera.resolution = (640, 480)
        self.camera.framerate = 32
        self.rawCapture = PiRGBArray(self.camera, size=(640,480))

    def get_img(self):
        self.camera.capture(self.rawCapture, format='bgr')
        cv_image = self.rawCapture.array
        try:
            image_message = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        except CVBridgeError as e:
            print(e)

        self.image_pub.publish(image_message)
        print('{CAMERA_NODE} Publishing! ')
        self.rawCapture.truncate(0)


if __name__ == '__main__':
    image_getter = ImagePublisher()
    rospy.init_node('image_getter', anonymous=True)
    loop = rospy.Rate(image_getter.node_rate)
    while not rospy.is_shutdown():
        image_getter.get_img()
        loop.sleep()

