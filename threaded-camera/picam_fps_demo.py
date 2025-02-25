from __future__ import print_function
from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import argparse
import imutils
import time
import cv2

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-n", "--num-frames", type=int, default=100, help="# of frames to loop over for FPS test")
ap.add_argument("-d", "--display", type=int, default=-1, help="Whether or not frames should be displayed")
args = vars(ap.parse_args())

# initialize the camera and stream
camera = PiCamera()
camera.resolution = (320, 240)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(320, 240))
stream = camera.capture_continuous(rawCapture, format="bgr", use_video_port=True)

print("[INFO] sampling THREADED frames from `picamera` modules...")
vs = PiVideoStream().start()
time.sleep(2.0)
fps = FPS().start()

#loop over some frames
while fps._numFrames < args["num_frames"]:
    # grab the frame from the threaded video stream and resize it to have a maximum width of 400 pixels
    frame = vs.read()
    frame = imutils.resize(frame, width=400)

    # check to see if the frame should be displayed to our screen
    if args["display"] > 0:
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF

    # update the FPS counter
    fps.update()

# stop the timer and display the FPS information
fps.stop()
print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
print("[INFO] approximate FPS: {:.2f}".format(fps.fps()))

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
