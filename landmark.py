#!/usr/bin/env python
import cv2
import numpy as np
from scipy import interpolate
import csv

class Landmark:

    def __init__(self):
        self.distance = 200
        self.qr_data = None
        self.detector = cv2.QRCodeDetector()
        self.x = np.array([])
        self.y = np.array([])
        file_name = open('landmark.csv', 'rt')
        reader = csv.reader(file_name)
        for row in reader:
            self.x = np.append(self.x, row[1])
            self.y = np.append(self.y, row[0])
        self.x = self.x.astype(float)
        self.y = self.y.astype(float)

    def get_qrdata(self, img):
        data, bbox, _ = self.detector.detectAndDecode(img)
        if data:  # aggiorno distanza e qr<-code quando vedo un landmark
            self.distance = self.get_distance(bbox)
            self.qr_data = data
        # se non ne vedo ritorno quella precedente
        return self.qr_data, self.distance

    def get_distance(self, points):
        x1 = points.item(0)
        x2 = points.item(2)
        y2 = points.item(3)
        y3 = points.item(5)
        h = abs(x1-x2)
        b = abs(y2-y3)
        area = h*b
        if area < 945.2836298234761 and area > 75.92754949163646:
            f = interpolate.interp1d(self.x, self.y)
            dist = f(area)
        else:
            dist = 200
        return dist







