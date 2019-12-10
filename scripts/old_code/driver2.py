import RPi.GPIO as GPIO 
import time

GPIO.setwarnings(False)

class Driver:

    def __init__(self, in1, in2, in3, in4, en1, en2):
        self.in1 = in1
        self.in2 = in2
        self.en1 = en1
        self.en2 = en2
        self.in3 = in3
        self.in4 = in4
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.en1, GPIO.OUT)
        GPIO.setup(self.in3, GPIO.OUT)
        GPIO.setup(self.in4, GPIO.OUT)
        GPIO.setup(self.en2, GPIO.OUT)
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        GPIO.output(self.in3, GPIO.LOW)
        GPIO.output(self.in4, GPIO.LOW)

    def move(self, distanza):
        tmp = -1
        p1 = GPIO.PWM(self.en1, 1000)
        p2 = GPIO.PWM(self.en2, 1000)
        p1.start(30)
        p2.start(30)
        if 80 < distanza <= 150:
            print("run")
            p1.ChangeDutyCycle(40)
            p2.ChangeDutyCycle(40)
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
            GPIO.output(self.in3, GPIO.HIGH)
            GPIO.output(self.in4, GPIO.LOW)
            time.sleep(0.8)

        elif 50 < distanza <= 80:
            print("forward")
            p1.ChangeDutyCycle(40)
            p2.ChangeDutyCycle(40)
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
            GPIO.output(self.in3, GPIO.HIGH)
            GPIO.output(self.in4, GPIO.LOW)
            time.sleep(0.5)

        elif 20 < distanza <= 50:
            print("forward slow")
            p1.ChangeDutyCycle(40)
            p2.ChangeDutyCycle(40)
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
            GPIO.output(self.in3, GPIO.HIGH)
            GPIO.output(self.in4, GPIO.LOW)
            time.sleep(0.2)

        elif 10 < distanza <= 20:
            print("almost there")
            p1.ChangeDutyCycle(35)
            p2.ChangeDutyCycle(35)
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
            GPIO.output(self.in3, GPIO.HIGH)
            GPIO.output(self.in4, GPIO.LOW)
            time.sleep(0.2)
        elif distanza == 900: #gira a dx
            print('gira a destra')
            p1.ChangeDutyCycle(35)
            p2.ChangeDutyCycle(35)
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
            GPIO.output(self.in3, GPIO.LOW)
            GPIO.output(self.in4, GPIO.HIGH)
            time.sleep(0.2)

        elif distanza == 800: #gira sx
            print('gira a sinistra')
            p1.ChangeDutyCycle(35)
            p2.ChangeDutyCycle(35)
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)
            GPIO.output(self.in3, GPIO.HIGH)
            GPIO.output(self.in4, GPIO.LOW)
            time.sleep(0.2)

        else: #target raggiunto
            print('target raggiunto')
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.LOW)
            GPIO.output(self.in3, GPIO.LOW)
            GPIO.output(self.in4, GPIO.LOW)
            GPIO.cleanup()
            tmp = 0

        return tmp

