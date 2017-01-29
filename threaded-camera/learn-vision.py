#from imutils.video.pivideostream import PiVideoStream
from BroncoCam import BroncoCam
from imutils.video import FPS
import imutils
from math import tan
import numpy as np
from networktables import NetworkTables
import time
import cv2

screenCnt = 0

resolution = (640, 480)
networkTablesServer = 'roborio-1987-frc.local'
NetworkTables.initialize(server=networkTablesServer)
visionTable = NetworkTables.getTable('SmartDashboard')

def findFocalLength(targetWidthInInches, targetWidthInPixels, targetDistanceInInches):
    focalLength = (targetWidthInPixels*targetDistanceInInches)/targetWidthInInches
    return focalLength

def findTargetWidthTimesFocalLength(targetWidthInInches):
    return targetWidthInInches * focalLength

def hsvThreshold(rgbImage, hue, sat, val):
    hsvImg = cv2.cvtColor(rgbImage, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsvImg, (hue[0], sat[0], val[0]), (hue[1], sat[1], val[1]))

def findContours(hsvImage):
    im1, contours, hierarchy = cv2.findContours(hsvImage, mode = cv2.RETR_EXTERNAL, method = cv2.CHAIN_APPROX_SIMPLE)
    return contours

def filterContours(contours, minWidth, maxWidth, minHeight, maxHeight):
    filteredContours = []
    for contour in contours:
        x,y,w,h = cv2.boundingRect(contour)
        if w >= minWidth and w <= maxWidth and h >= minHeight and h <= maxHeight:
            filteredContours.append(contour)
    return filteredContours

def findBoundingRectOfContour(contours, minWidth, maxWidth, minHeight, maxHeight):
    filteredRects = []
    for contour in contours:
        x,y,w,h = cv2.boundingRect(contour)
        if w >= minWidth and w <= maxWidth and h >= minHeight and h <= maxHeight:
            filteredRects.append((x,y,w,h))
    return (filteredRects)

def drawContours(hsvImage, c):
    return cv2.drawContours(hsvImage, c, -1, (255, 255, 0), 4)

def detectShape(hsvImage, c):
    for s in c:
        perm = cv2.arcLength(s, True)
        approx = cv2.approxPolyDP(s, 0.02 * perm, True)
        #print(perm)
        return (perm)
        print(len(approx))
        if len(approx) == 4: 
            screenCnt = approx
            return screenCnt

def angleToTarget(targetX, targetWidth):
    hov = 62.2 # angle of horizontal view in degrees
    degreesPerPixel = 62.2 / resolution[0]
    centerOfImage = resolution[0] / 2
    return ((targetX + targetWidth/2) - centerOfImage) * degreesPerPixel

def distanceToTarget(targetWidth):
    return targetWidthTimesFocalLength / targetWidth

focalLength = findFocalLength(20, 190, 136)
targetWidthTimesFocalLength = findTargetWidthTimesFocalLength(20)

videoStream = BroncoCam(resolution, 60).start()
time.sleep(1.0)
fps = FPS().start()

"""hue = [110, 130]
sat = [50, 255]
val = [50, 255]"""

hue = [80, 160]
sat = [30, 255]
val = [30, 255]

while fps._numFrames < 1000:                #quits after seconds
    frame = videoStream.read()
    cv2.imshow("Frame", frame)
    hsvImage = hsvThreshold(frame, hue, sat, val)
    c = findContours(hsvImage)
    if len(c) <= 0:
        continue;#contours, minWidth, maxWidth, minHeight, maxHeight
    fc = filterContours(c, 50, 600, 50, 400)
    fb = findBoundingRectOfContour(c, 100, 600, 100, 400)
    if len(fb) > 0:
        #boxX, boxY, boxW, boxH = cv2.boundingRect(fc[0])
        angle = angleToTarget(fb[0][0], fb[0][2])
        distance = distanceToTarget(fb[0][2])
        #print("angle:", angle)
        #print("distance:", distance)
        visionTable.putNumber('angle', angle)
        visionTable.putNumber('distance', distance)
    a = drawContours(hsvImage, fc)
    #cv2.imshow("contours", a)
    #cv2.imshow("Frame", frame)
    #cv2.imshow("HSV", hsvImage)
    key = cv2.waitKey(1) & 0xFF
    fps.update()
    
fps.stop()

print("FPS: {:.2f}".format(fps.fps()))

cv2.destroyAllWindows()
videoStream.stop()
