import csv
from picamera import PiCamera
from picamera.array import PiRGBArray
from dynamic_distance import Distance
import time
import sys
import cv2


camera = PiCamera()
#camera.resolution = (640, 480)
#camera.framerate = 32
rawCapture = PiRGBArray(camera)
time.sleep(0.2) 
f = open(sys.argv[1], 'wt')
writer = csv.writer(f)
i = 0
while i < 150:
    camera.capture(rawCapture, format='bgr')
    imm = rawCapture.array
    if imm is not None:
        area = Distance.find_area(imm)
        i = i + 10
        writer.writerow((i, area))
    cv2.waitKey(0)
    rawCapture.truncate(0)

