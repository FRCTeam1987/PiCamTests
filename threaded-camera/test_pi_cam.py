from imutils.video.pivideostream import PiVideoStream
#from imutils.video import PiVideoStream
import imutils
import numpy as np
import cv2
import time

screenCnt = 0

resolution = (640, 480)

videoStream = PiVideoStream(resolution, 60).start()
time.sleep(1.0)

while True:                #quits after seconds
    frame = videoStream.read()
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

cv2.destroyAllWindows()
videoStream.stop()
