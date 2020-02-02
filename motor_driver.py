#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
import time


class MotorDriver(object):
    
    def __init__(self):
        
        self.wheel_distance = 0.158
        self.wheel_radius = 0.0172
        self.PWM1 = 0
        self.PWM2 = 0
        self.in1 = 27
        self.in2 = 17
        self.in3 = 6
        self.in4 = 5
        self.en1 = 12
        self.en2 = 13
        
        self.BASE_PWM = 40
        self.MAX_PWM = 100
        
        self.MULTIPLIER_STANDARD = 0.3
        self.MULTIPLIER_PIVOT = 0.3
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.en1, GPIO.OUT)
        GPIO.setup(self.in3, GPIO.OUT)
        GPIO.setup(self.in4, GPIO.OUT)
        GPIO.setup(self.en2, GPIO.OUT)
        
        self.p1 = GPIO.PWM(self.en1, 1000)
        self.p2 = GPIO.PWM(self.en2, 1000)
        
        self.p1.start(self.PWM1)
        self.p2.start(self.PWM2)

        self.change_speed(0.05, 0)
        
    def __del__(self):
        GPIO.cleanup()
        
    def set_motor(self, v1, v2, v3, v4):
        GPIO.output(self.in1, v1)
        GPIO.output(self.in2, v2)
        GPIO.output(self.in3, v3)
        GPIO.output(self.in4, v4)
        
    def forward(self):
        self.set_motor(GPIO.HIGH, GPIO.LOW, GPIO.HIGH, GPIO.LOW)

    def stop(self):
        self.set_motor(GPIO.LOW, GPIO.LOW, GPIO.LOW, GPIO.LOW)
     
    def reverse(self):
        self.set_motor(GPIO.LOW, GPIO.HIGH, GPIO.LOW, GPIO.HIGH)

    def pivot_right(self):
        self.set_motor(GPIO.HIGH, GPIO.LOW, GPIO.LOW, GPIO.HIGH)

    def right(self):
        self.set_motor(GPIO.HIGH, GPIO.LOW, GPIO.LOW, GPIO.LOW)

    def pivot_left(self):
        self.set_motor(GPIO.LOW, GPIO.HIGH, GPIO.HIGH, GPIO.LOW)
        
    def left(self):
        self.set_motor(GPIO.LOW, GPIO.LOW, GPIO.HIGH, GPIO.LOW)
        
    def left_reverse(self):
        self.set_motor(GPIO.LOW, GPIO.HIGH, GPIO.LOW, GPIO.LOW)
        
    def right_reverse(self):
        self.set_motor(GPIO.LOW, GPIO.LOW, GPIO.LOW, GPIO.HIGH)
        
    def set_speed(self, rpm_speed_1, rpm_speed_2, multiplier):
        self.PWM1= min(int(rpm_speed_1*multiplier*self.BASE_PWM), self.MAX_PWM)
        self.PWM2= min(int(rpm_speed_2*multiplier*self.BASE_PWM), self.MAX_PWM)
        self.p1.ChangeDutyCycle(self.PWM1)
        self.p2.ChangeDutyCycle(self.PWM2)

    def calculate_body_turn_radius(self, linear_speed, angular_speed):
        if angular_speed != 0.0:
            body_turn_radius = linear_speed/angular_speed
        else:
            body_turn_radius = None
        return body_turn_radius

    def calculate_wheel_turn_radius(self, body_turn_radius, wheel):
        if body_turn_radius is not None:
            if wheel == 'right':
                wheel_sign = 1
            else:
                wheel_sign = -1
                
            wheel_turn_radius = body_turn_radius +(wheel_sign * (self.wheel_distance / 2.0))
        else:
            wheel_turn_radius = None
        return wheel_turn_radius

    def calculate_wheel_rpm(self, linear_speed, angular_speed, wheel_turn_radius):
        if wheel_turn_radius is not None:
            wheel_rpm = (angular_speed * wheel_turn_radius) / self.wheel_radius
        else:
            
            wheel_rpm = linear_speed / self.wheel_radius
            
        return wheel_rpm

    def set_wheel_movement(self, right_wheel_rpm, left_wheel_rpm):
        # sulla base del fatto che i due valori passati come argomento siano positivi, negativi o
        # nulli andiamo a passare dei valori alti o bassi ai pin GPIO
        # usiamo il valore assoluto perche non possiamo passare valori negativi a set_speed
        # visto che servono per variare il duty cycle
        if right_wheel_rpm > 0.0 and left_wheel_rpm > 0.0:
            self.set_speed(abs(right_wheel_rpm), abs(left_wheel_rpm), self.MULTIPLIER_STANDARD)
            self.forward()
                
        elif right_wheel_rpm > 0.0 and left_wheel_rpm == 0.0:
            self.set_speed(abs(right_wheel_rpm), abs(left_wheel_rpm), self.MULTIPLIER_STANDARD)
            self.left()
        
        elif right_wheel_rpm > 0.0 and left_wheel_rpm < 0.0:
            self.set_speed(abs(right_wheel_rpm), abs(left_wheel_rpm), self.MULTIPLIER_PIVOT)
            self.pivot_left()
        
            
        elif right_wheel_rpm == 0.0 and left_wheel_rpm > 0.0:
            self.set_speed(abs(right_wheel_rpm), abs(left_wheel_rpm), self.MULTIPLIER_STANDARD)
            
            self.right()
            
        elif right_wheel_rpm < 0.0 and left_wheel_rpm > 0.0:
            self.set_speed(abs(right_wheel_rpm), abs(left_wheel_rpm), self.MULTIPLIER_PIVOT)
            
            self.pivot_right()
                
        elif right_wheel_rpm < 0.0 and left_wheel_rpm < 0.0:
            self.set_speed(abs(right_wheel_rpm), abs(left_wheel_rpm), self.MULTIPLIER_STANDARD)
            
            self.reverse()
            
        elif right_wheel_rpm == 0.0 and left_wheel_rpm == 0.0:
            self.set_speed(abs(right_wheel_rpm), abs(left_wheel_rpm), self.MULTIPLIER_STANDARD)
            self.stop()
        
        else:
            pass
        
    def change_speed(self, ls, a_s):
        # stabiliamo se il cerchio che dobbiamo percorrere  e piccolo o grande
        body_turn_radius = self.calculate_body_turn_radius(ls, a_s)
        
        wheel = 'right'
        # calcoliamo sulla base di quanto deve girare il robot quale  il raggio della curva delle signole ruote
        right_wheel_turn_radius = self.calculate_wheel_turn_radius(body_turn_radius, wheel)
        wheel = 'left'
        left_wheel_turn_radius = self.calculate_wheel_turn_radius(body_turn_radius, wheel)
        
        # calcoliamo la velocita di rotazione delle singole ruote
        right_wheel_rpm = self.calculate_wheel_rpm(ls, a_s, right_wheel_turn_radius)    
        left_wheel_rpm = self.calculate_wheel_rpm(ls, a_s, left_wheel_turn_radius)
        
        self.set_wheel_movement(right_wheel_rpm, left_wheel_rpm)
        
            
            
        
        





