import numpy as np
import math
import cv2 as cv
from picamera import PiCamera
from picamera.array import PiRGBArray
import RPi.GPIO as GPIO
import func
import time

IN1 = 1
IN2 = 7
IN3 = 8
IN4 = 25
ENA = 12
ENB = 13
SRV = 17

RECT = np.float32([[0, 299],
                   [399, 299],
                   [399, 0],
                   [0, 0]])

TRAP = np.float32([[0, 299],
                   [399, 299],
                   [320, 200],
                   [80, 200]])

class Robot:
    def __init__(self):
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
        self.ml.start(0)

        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        self.mr = GPIO.PWM(ENB, 1000)
        self.mr.stop()
        self.mr.start(0)
        
        self.erld = 0
        
    def servo_move(self, angle):
        pwm = GPIO.PWM(SRV, 50)
        pwm.start(8)
        data = angle / 18.0 + 3.0
        pwm.ChangeDutyCycle(data)
        time.sleep(0.5)
        pwm.stop()

    def detect_edges(self, hsv):
        lower_blue = np.array([90, 120, 0], dtype="uint8")
        upper_blue = np.array([150, 255, 255], dtype="uint8")
        mask = cv.inRange(hsv,lower_blue,upper_blue)
        edges = cv.Canny(mask, 50, 100) 
        return edges

    def region_of_interest(self, edges):
        height, width = edges.shape
        mask = np.zeros_like(edges)

        polygon = np.array([[
            (0, height), 
            (0,  height/2),
            (width , height/2),
            (width , height),
        ]], np.int32)

        cv.fillPoly(mask, polygon, 255)
        cropped_edges = cv.bitwise_and(edges, mask) 
        return cropped_edges

    def detect_line_segments(self, cropped_edges):
        rho = 1  
        theta = np.pi / 180  
        min_threshold = 10
        line_segments = cv.HoughLinesP(cropped_edges, rho, theta, min_threshold, np.array([]), minLineLength=5, maxLineGap=0)
        return line_segments

    def average_slope_intercept(self, frame, line_segments):
        lane_lines = []

        if line_segments is None:
            return lane_lines

        height, width,_ = frame.shape
        left_fit = []
        right_fit = []
        boundary = 1/3

        left_region_boundary = width * (1 - boundary) 
        right_region_boundary = width * boundary 
        for line_segment in line_segments:
            for x1, y1, x2, y2 in line_segment:
                if x1 == x2:
                    continue

                fit = np.polyfit((x1, x2), (y1, y2), 1)
                slope = (y2 - y1) / (x2 - x1)
                intercept = y1 - (slope * x1)

                if slope < 0:
                    if x1 < left_region_boundary and x2 < left_region_boundary:
                        left_fit.append((slope, intercept))
                else:
                    if x1 > right_region_boundary and x2 > right_region_boundary:
                        right_fit.append((slope, intercept))

        left_fit_average = np.average(left_fit, axis=0)
        if len(left_fit) > 0:
            lane_lines.append(self.make_points(frame, left_fit_average))

        right_fit_average = np.average(right_fit, axis=0)
        if len(right_fit) > 0:
            lane_lines.append(self.make_points(frame, right_fit_average))

        return lane_lines

    def make_points(self, frame, line):
        height, width, _ = frame.shape
        slope, intercept = line
        y1 = height
        y2 = int(y1 / 2)

        if slope == 0: 
            slope = 0.1    

        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)

        return [[x1, y1, x2, y2]]

    def get_steering_angle(self, frame, lane_lines):
        height, width, _ = frame.shape
         
        if len(lane_lines) == 2:
            _, _, left_x2, _ = lane_lines[0][0]
            _, _, right_x2, _ = lane_lines[1][0]
            mid = int(width / 2)
            x_offset = (left_x2 + right_x2) / 2 - mid
            y_offset = int(height / 2)  
        elif len(lane_lines) == 1:
            x1, _, x2, _ = lane_lines[0][0]
            x_offset = x2 - x1
            y_offset = int(height / 2)
        
        elif len(lane_lines) == 0:
            x_offset = 0
            y_offset = int(height / 2)

        angle_to_mid_radian = math.atan(x_offset / y_offset)
        angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  
        steering_angle = angle_to_mid_deg + 90 

        return steering_angle

    def move(self, vl, vr):
        if abs(vl) >= 100:
            vl = vl // abs(vl) * 100
        if abs(vr) >= 100:
            vr = vr // abs(vr) * 100

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

        up = err * 0.2 + (err - self.erld) * 0.1
        self.erld = err
        vl = speed + up
        vr = speed - up

        self.move(vl, vr)

    def detect_color(self, img, color):
        img = cv.resize(img, (400, 300))
        if color == "red":
            lower = [0, 0, 100]
            upper = [90, 90, 220]
        elif color == "yellow":
            lower = [0, 150, 120]
            upper = [130, 220, 220]
        elif color == "green":
            lower = [0, 100, 0]
            upper = [100, 220, 120]
        elif color == "brown":
            lower = [0, 60, 100]
            upper = [80, 120, 220]

        img = cv.inRange(img, np.array(lower), np.array(upper))
        cv.imshow("123", img)

        s = np.sum(img)
        if s > 1000000:
            return (True, img)
        return (False, [])

    def detect_end(self, img):
        lower = [130, 70, 0]
        upper = [240, 180, 90]
        img = cv.inRange(img, np.array(lower), np.array(upper))

        s = np.sum(img)
        if s > 2000000:
            return True
        return False

    def center(self, img):
        ctrs, _ = cv.findContours(img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        rect = cv.boundingRect(max(ctrs, key=cv.contourArea))

        cube_x = rect[0]+rect[2]//2
        err = 200 - cube_x
        up = err * 0.2 + (err - self.erld) * 0.3
        self.erld = err
        self.move(-up, up)

        return err

    def close(self):
        self.servo_move(0)

    def open(self):
        self.servo_move(90)
