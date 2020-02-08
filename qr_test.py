import cv2
import numpay as np


def video_reader():
    cam = cv2.VideoCapture(0)
    detector = cv2.QRCodeDetector()
    while True:
        _, img = cam.read()
        data, points, _ = detector.detectAndDecode(img)
        if data:
            print("QR Code detected-->", data)
            x1 = points.item(0)
            x2 = points.item(2)
            y2 = points.item(3)
            y3 = points.item(5)
            h = abs(x1 - x2)
            b = abs(y2 - y3)
            print("Area: " + str(h * b))
        cv2.imshow("img", img)
        if cv2.waitKey(1) == ord("Q"):
            break
    cam.release()
    cv2.destroyAllWindows()


video_reader()
