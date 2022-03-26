import cv2 as cv
from picamera import PiCamera
from picamera.array import PiRGBArray
import time
import robot
import random

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.1)

robot = robot.Robot()
robot.open()
trig = 0
trig_move = -1
trig_im = []
trig_time = 0
trig_end = 0
count_r = 0
count_y = 0
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array
    cv.imshow("1", image)

    if trig <= 0:
        if robot.detect_end(image):
            robot.move(-40, -40)
            time.sleep(0.5)
            trig = -1
            robot.move(50 * trig_move, 50 * -trig_move)
        else:
            if trig == -1:
                trig = 0
                time.sleep(random.choice([0.3, 0.5, 0.6, 0.8, 1.0, 1.2]))
                trig_move = trig_move

        if trig == 0:
            robot.move(30, 30)
            (red, red_im) = robot.detect_color(image, "red")
            (yellow, yellow_im) = robot.detect_color(image, "yellow", 1)

            if red:
                print("red")
                trig = 1
                robot.move(0, 0)
                time.sleep(1)
            elif yellow:
                print("yellow")
                trig = 2
                robot.move(0, 0)
                time.sleep(1)
    else:
        if trig == 1 or trig == 2:
            (b, im) = robot.detect_color(image, "red" if trig == 1 else "yellow")
            if b:
                if robot.center(im):
                    trig += 2
            else:
                trig = 0
        elif trig == 3 or trig == 4:
            robot.move(30, 30)
            time.sleep(1.5)
            robot.move(0, 0)
            time.sleep(0.5)
            robot.close()
            time.sleep(1)
            robot.move(-50, -50)
            time.sleep(0.6)
            robot.move(40, 40)
            trig += 2
        elif trig == 5 or trig == 6:
            if robot.detect_end(image):
                robot.move(-40, -40)
                time.sleep(0.5)
                robot.move(-50, 50)
                trig += 2
        elif trig == 7 or trig == 8:
            if robot.detect_color(image, "green", 1)[0]:
                if trig == 7:
                    trig = 11

            if robot.detect_color(image, "black")[0]:
                if trig == 8:
                    trig = 12

            if trig_end == 1 and time.time() - trig_time > 0.5:
                robot.trig = 0
                robot.time = time.time()
                trig += 2
            elif not robot.detect_end(image) and trig_end == 0:
                trig_time = time.time()
                trig_end = 1
            elif robot.detect_end(image):
                trig_time = time.time()
                trig_end = 0
        elif trig == 9 or trig == 10:
            robot.line(image, 30, "right")
            
            if robot.detect_color(image, "green", 1)[0]:
                if trig == 9:
                    trig = 11

            if robot.detect_color(image, "black")[0]:
                if trig == 10:
                    trig = 12
        elif trig == 11 or trig == 12:
            robot.move(50, 50)
            time.sleep(0.3)
            robot.move(0, 0)
            robot.open()
            time.sleep(1.0)
            robot.move(-50, -50)
            time.sleep(0.2)
            robot.move(-50, 50)
            time.sleep(0.6)
            if trig == 11:
                count_r += 1
            elif trig == 12:
                count_y += 1
            trig = 0

    rawCapture.truncate(0)
    if cv.waitKey(1) == 27 or (count_r >= 3 and count_y >= 3):
        robot.ml.stop()
        robot.mr.stop()
        break
