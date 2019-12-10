#!/usr/bin/env python
#import sys
import rospy
from std_msgs.msg import String
import numpy as np
import cv2
from scipy import interpolate
import csv
from picamera import PiCamera
from picamera.array import PiRGBArray


def find_area(image):
    image_blur = cv2.GaussianBlur(image, (5, 5), 0)
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # cv2.imshow('Image', img_hsv)
    #cv2.waitKey(6000)
    lowgreen = np.array([35, 125, 70])
    highgreen = np.array([50, 255, 255])

    green_mask = cv2.inRange(img_hsv, lowgreen, highgreen)
    mask_filter = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    edged_img = cv2.Canny(mask_filter.copy(), 35, 125)
    # cv2.imshow('Edged', edged_img)
    #cv2.waitKey(2000)
    cnts, hierarchy = cv2.findContours(edged_img.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return 0, 0
    else:
        if len(cnts) > 0:
            largest = 0
            area = 0
            for i in range(len(cnts)):
                # get the area of the ith contour
                temp_area = cv2.contourArea(cnts[i])
                # if it is the biggest we have seen, keep it
                if temp_area > area:
                    area = temp_area
                    largest = i
            # Compute the x coordinate of the center of the largest contour
            coordinates = cv2.moments(cnts[largest])
            target_x = int(coordinates['m10'] / coordinates['m00'])
    return area, target_x

def distancetoCamera(sup):
    x = np.array([])
    y = np.array([])
    file_name = open('Misure.csv', 'rt')
    reader = csv.reader(file_name)
    for row in reader:
        x = np.append(x, row[1])
        y = np.append(y, row[0])
    x = x.astype(float)
    y = y.astype(float)
    if sup < 131395.5 and sup > 2494:
        f = interpolate.interp1d(x, y)
        return f(sup)
    else: 
        return 0

def main():
    cam = PiCamera()
    #cam.resolution(640, 480)
    rawCapture = PiRGBArray(cam)
    cam.capture(rawCapture, format='bgr')
    imm = rawCapture.array

    pub = rospy.Publisher('distance', String, queue_size=10)
    rospy.init_node('dist_find', anonymous=True)
    rate = rospy.Rate(10)
    while not ((rospy.is_shutdown()) or (imm is None)):

        x,a = find_area(imm)
        y = distancetoCamera(x)
        str = "Area: %i, Distanza: %i, Target: %i" 
        print(str)
        rospy.loginfo(str)
        pub.publish(str)
        rate.sleep()

        #print("Area: ")
        #print(x)
        #print("; Distanza: ")
        #print(y)
        #print("; target_x: ")
        #print(a)

if __name__ == '__main__':
    main()
