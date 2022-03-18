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

robot = robot.Robot(0.4, 0.05)
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array
    cv.imshow("1", image)

    rawCapture.truncate(0)
    
    if cv.waitKey(1) == 27:
        robot.ml.stop()
        robot.mr.stop()
        break
