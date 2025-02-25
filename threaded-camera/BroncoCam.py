from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
import cv2

class BroncoCam:
    def __init__(self, resolution=(320,240), framerate=60):
        self.camera = PiCamera()
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        self.camera.shutter_speed = 100
        self.camera.awb_mode = "off"
        self.camera.awb_gains = (0.1, 1.9) #ignored if awb_mode is not off
        self.rawCapture = PiRGBArray(self.camera, size=resolution)
        self.stream = self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True)
        self.frame = None
        self.stopped = False
        self.camera.brightness = 50
        self.camera.hflip = True #when camera is upside down
        self.camera.rotation = 180 #when camera is upside down
        self.camera.exposure_mode = "fireworks"
        #self.camera.iso = 200
        print("Exposure Speed:", self.camera.exposure_speed)
        #print("Brightness:",self.camera.brightness)
    def start(self):
        t = Thread(target=self.update, args=())
        t.daemon = True
        t.start()
        return self
    def update(self):
        for f in self.stream:
            self.frame = f.array
            self.rawCapture.truncate(0)
            if self.stopped:
                self.stream.close()
                self.rawCapture.close()
                self.camera.close()
                return
    def read(self):
        return self.frame
    def stop(self):
        self.stopped = True
