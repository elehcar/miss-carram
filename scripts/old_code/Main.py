# coding=utf-8
#!/usr/bin/env python
from picamera import PiCamera
from picamera.array import PiRGBArray
import time
from dynamic_distance import Distance
from driver2 import Driver
import cv2


def main():
    # creo l'oggetto della classe Camera e assegno 3 pin ad ogni ruota, il terzo è il pin
    # per PWM (12,13,18,19)
    w = 640
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640,480))
    time.sleep(0.2)
    driver = Driver(24, 23, 21, 20, 12, 13)
    d = Distance()
    tmp = 1
     # ricavo l'area dell'oggetto nell'immagine
    while True:
    #for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
        camera.capture(rawCapture, format='bgr')
        image = rawCapture.array
        area, target_x, img_cnt = Distance.find_area(image)
        #area, target_x = Distance.find_area(image)
        dist = d.distancetoCamera(area)
        print(area)
        print(dist)
        print(target_x)
        if dist != 200: #target visibile
            if dist >120:
                range= 40
            elif  80 < dist < 120:
                range = 45
            elif 80 <= dist < 50:
                range=80
            elif 50<= dist < 30:
                range=100
            else:
                range = 120
            # devo gestire la rotazione a destra o a sx basandomi sulle informazioni in coordinate
            # restituite dalla find contours.
            if target_x < (w/2-range):
                print('target a sx rispetto al centro')
                t = driver.move(800)
            elif target_x > (w/2+range):
                print('target a dx rispetto al centro')
                t = driver.move(900)
                tmp = 2
                # una volta che ho verificato che l'oggetto è al centro della camera
                # procedo dritto secondo una velocità determinata sulla base della distanza calcolata
                # con la funzione distancetoCamera
            else:  # w/2-10 < target_x < w/2+10
                print('target al centro')
                t = driver.move(dist)
        else: #dist == 150 ovvero area nulla o molto piccola
            print('target non visibile')
            if tmp == 1:
                t = driver.move(800)
            else:
                t = driver.move(900)
                tmp = 1
        cv2.imshow('Immagine', img_cnt)
        cv2.waitKey(500)
        rawCapture.truncate(0)
        # premere esc per stoppare, altrimenti continua dopo 1 millisecondo
        key_pressed = cv2.waitKey(1)
        if key_pressed == 27 or t == 0:
            break


if __name__ == '__main__':
    main()
