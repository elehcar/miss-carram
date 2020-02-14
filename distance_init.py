import csv
import sys
import cv2
import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray
import time

def find_area(image):
    image_blur = cv2.GaussianBlur(image, (5, 5), 0)
    img_hsv = cv2.cvtColor(image_blur, cv2.COLOR_BGR2HSV)
    # lowred = np.array([0, 57, 82])
    # highred = np.array([21, 255, 255])

    lowblue = np.array([80, 70, 20])
    highblue = np.array([120, 255, 255])
    green_mask = cv2.inRange(img_hsv, lowblue, highblue)
    mask_filter = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    edged_img = cv2.Canny(mask_filter.copy(), 35, 125)
    # cv2.imshow('Edged', edged_img)
    # cv2.waitKey(1000)
    cnts, hierarchy = cv2.findContours(edged_img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
            img_cnts = cv2.drawContours(image.copy(), cnts[largest], -1, (40, 255, 255))
            # cv2.imshow('Immagine',img_cnts)
            # cv2.waitKey(1000)
            if coordinates["m00"] != 0:
                target_x = int(coordinates['m10'] / coordinates['m00'])
            else:
                target_x = 0
    # return area, target_x, img_cnts
    return area, target_x

def get_qrarea(image):
    detector = cv2.QRCodeDetector()
    data, points, _ = detector.detectAndDecode(image)
    area_qr = 0
    if data:
        x1 = points.item(0)
        x2 = points.item(2)
        y2 = points.item(3)
        y3 = points.item(5)
        h = abs(x1 - x2)
        b = abs(y2 - y3)
        area_qr = h * b
    return area_qr

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(2)
f = open(sys.argv[1], 'wt')
writer = csv.writer(f)
j = 20
while j < 150:
    camera.capture(rawCapture, format='bgr')
    imm = rawCapture.array
    if imm is not None:
        area = get_qrarea(imm)
        j = j + 10
        writer.writerow((j, area))
    cv2.imshow('Imm', imm)
    cv2.waitKey(0)
    rawCapture.truncate(0)

