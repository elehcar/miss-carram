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

    def aux(self, cnts, image):
        area = 0
        target_x = 0
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
        return area, target_x

    def find_area(self, image):

        # cv2.imshow('Im', image)
        # cv2.waitKey(0)
        final_area = 0
        final_target = 0

        image_blur = cv2.GaussianBlur(image, (5, 5), 0)
        img_hsv = cv2.cvtColor(image_blur, cv2.COLOR_BGR2HSV)

        lowred = np.array([0, 57, 82])
        highred = np.array([21, 255, 255])

        lowgreen = np.array([41, 120, 80])
        highgreen = np.array([55, 255, 255])

        lowblue = np.array([80, 70, 20])
        highblue = np.array([120, 255, 255])

        green_mask = cv2.inRange(img_hsv, lowgreen, highgreen)
        blue_mask = cv2.inRange(img_hsv, lowblue, highblue)
        red_mask = cv2.inRange(img_hsv, lowred, highred)

        mask_filter_green = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask_filter_red = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask_filter_blue = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        edged_img_green = cv2.Canny(mask_filter_green.copy(), 35, 125)
        edged_img_red = cv2.Canny(mask_filter_red.copy(), 35, 125)
        edged_img_blue = cv2.Canny(mask_filter_blue.copy(), 35, 125)

        # cv2.imshow('Image', edged_img_red)
        # cv2.waitKey(0)
        area_results = np.array([])
        target_results = np.array([])

        # cv2.imshow('Edged', edged_img)
        # cv2.waitKey(1000)
        cnts_green, hierarchy_green = cv2.findContours(edged_img_green.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts_red, hierarchy_red = cv2.findContours(edged_img_red.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts_blue, hierarchy_blue = cv2.findContours(edged_img_blue.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        area_green, target_green = self.aux(cnts_green, image)
        area_blue, target_blue = self.aux(cnts_blue, image)
        area_red, target_red = self.aux(cnts_red, image)

        area_results = np.append(area_results, area_green)
        area_results = np.append(area_results, area_blue)
        area_results = np.append(area_results, area_red)

        target_results = np.append(target_results, target_green)
        target_results = np.append(target_results, target_blue)
        target_results = np.append(target_results, target_red)

        for i in area_results:
             final_area = 0
             if i > final_area:
                 final_area = i
                 index = np.where(area_results == i)
                 final_target = target_results[index]

        return final_area, final_target

    def distancetoCamera(self, sup):
        if sup > 111459.0:  # ostacolo imminente
            return 0
        elif sup < 3937.5:  # ostacolo lontano
            return 200
        else:
            f = interpolate.interp1d(self.x, self.y)
            return f(sup)

