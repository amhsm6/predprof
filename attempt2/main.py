import cv2 as cv
from picamera import PiCamera
from picamera.array import PiRGBArray
import time
import robot

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.1)

robot = robot.Robot()
trig = 0
trig_im = []
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array
    cv.imshow("1", image)

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
                if err < 10:
                    trig = 3 if trig == 1 else 4
            else:
                trig = 0
        else:
            robot.move(30, 30);
            time.sleep(0.5)
            robot.stop()
            break

    rawCapture.truncate(0)
    if cv.waitKey(1) == 27:
        robot.stop()
        break
