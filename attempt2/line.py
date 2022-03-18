import numpy as np
import cv2 as cv
import math
import RPi.GPIO as GPIO

IN1 = 1
IN2 = 7
IN3 = 8
IN4 = 25
ENA = 12
ENB = 13
SRV = 18

class Robot:
    def __init__(self, kp, kd):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(IN1, GPIO.OUT)
        GPIO.setup(IN2, GPIO.OUT)
        GPIO.setup(IN3, GPIO.OUT)
        GPIO.setup(IN4, GPIO.OUT)
        GPIO.setup(ENA, GPIO.OUT)
        GPIO.setup(ENB, GPIO.OUT)
        GPIO.setup(SRV, GPIO.OUT)

        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        self.ml = GPIO.PWM(ENA, 1000) 
        self.ml.stop()
        self.ml.start(10)

        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        self.mr = GPIO.PWM(ENB, 1000)
        self.mr.stop()
        self.mr.start(10)

        self.srv = GPIO.PWM(SRV, 50)
        self.srv.start(0)

        self.kp = kp
        self.kd = kd
        
        self.erld = 0

    def line(self, img, speed, way):
        img = cv.resize(img, (400, 300))
        binary = func.binarize(img)
        perspective = func.trans_perspective(binary, TRAP, RECT, (400, 300))
        left, right = func.centre_mass(perspective, d=1)

        err = 0 - ((left + right) // 2 - 200)
        if way == 'right':
            err = 280 - right
        elif way == 'left':
            err = 120 - left

        up = err * self.kp + (err - self.erld) * self.kd
        self.erld = err
        vl = speed + up
        vr = speed - up
        
        if abs(vl) >= 100:
            vl = vl // abs(vl)
        if abs(vr) >= 100:
            vr = vr // abs(vr)

        if vl < 0:
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.HIGH)
            vl = -vl
        else:
            GPIO.output(IN1, GPIO.HIGH)
            GPIO.output(IN2, GPIO.LOW)

        if vr < 0:
            GPIO.output(IN3, GPIO.LOW)
            GPIO.output(IN4, GPIO.HIGH)
            vr = -vr
        else:
            GPIO.output(IN3, GPIO.HIGH)
            GPIO.output(IN4, GPIO.LOW)

        self.ml.ChangeDutyCycle(vl)
        self.mr.ChangeDutyCycle(vr)
