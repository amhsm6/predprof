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
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array
    cv.imshow("1", image)

    if trig <= 0:
        if robot.detect_end(image):
            trig = -1
            robot.move(50 * trig_move, 50 * -trig_move)
        else:
            if trig == -1:
                trig = 0
                time.sleep(random.choice([0.3, 0.5, 0.6, 0.8, 1.0, 1.2]))
                trig_move = trig_move

        if trig == 0:
            robot.move(20, 20)
            (red, red_im) = robot.detect_color(image, "red")
            (yellow, yellow_im) = robot.detect_color(image, "yellow")

            if red:
                print("red")
                trig = 1
            elif yellow:
                print("yellow")
                trig = 2
    else:
        if trig == 1 or trig == 2:
            (b, im) = robot.detect_color(image, "red" if trig == 1 else "yellow")
            if b:
                err = robot.center(im)
                if abs(err) < 15:
                    trig += 2
            else:
                trig = 0
        elif trig == 3 or trig == 4:
            robot.move(50, 50)
            time.sleep(0.6)
            robot.move(0, 0)
            time.sleep(0.5)
            robot.close()
            time.sleep(1)
            robot.move(50, 50)
            trig += 2
        elif trig == 5 or trig == 6:
            if robot.detect_end(image):
                robot.move(-50, 50)
                trig += 2
        elif trig == 7 or trig == 8:
            if not robot.detect_end(image):
                time.sleep(0.1)
                trig += 2
        elif trig == 9 or trig == 10:
            robot.line(image, 30, "right")
            
            #if robot.detect_color(image, "green"):
            #    if trig == 9:
            #        trig == 11

            if robot.detect_color(image, "brown"):
                if trig == 10:
                    trig == 12
        elif trig == 11 or trig == 12:
            robot.move(50, 50)
            time.sleep(0.3)
            robot.move(0, 0)
            robot.open()
            time.sleep(1.0)
            robot.move(-50, -50)
            time.sleep(0.2)
            robot.move(-50, 50)
            time.sleep(0.4)
            robot.move(50, 50)
            time.sleep(2)

    rawCapture.truncate(0)
    if cv.waitKey(1) == 27:
        robot.ml.stop()
        robot.mr.stop()
        robot.srv.stop()
        break
