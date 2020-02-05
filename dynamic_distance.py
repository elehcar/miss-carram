import numpy as np
import cv2
from scipy import interpolate
import csv


class Distance:

    def __init__(self):

        self.x = np.array([])
        self.y = np.array([])
        file_name = open('Obstacle_little.csv', 'rt')
        reader = csv.reader(file_name)
        for row in reader:
            self.x = np.append(self.x, row[1])
            self.y = np.append(self.y, row[0])
        self.x = self.x.astype(float)
        self.y = self.y.astype(float)

    def find_area(self, image):
        image_blur = cv2.GaussianBlur(image, (5, 5), 0)
        img_hsv = cv2.cvtColor(image_blur, cv2.COLOR_BGR2HSV)
        lowblue = np.array([80, 70, 20])
        highblue = np.array([120, 255, 255])

        blue_mask = cv2.inRange(img_hsv, lowblue, highblue)
        mask_filter = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

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
                if coordinates['m00'] != 0:
                    target_x = int(coordinates['m10'] / coordinates['m00'])
                else:
                    target_x = 0
        # return area, target_x, img_cnts
        return area, target_x

    def distancetoCamera(self, sup):
        if sup > 111459.0:  # ostacolo imminente
            return 0
        elif sup < 3937.5:  # ostacolo lontano
            return 200
        else:
            f = interpolate.interp1d(self.x, self.y)
            return f(sup)

