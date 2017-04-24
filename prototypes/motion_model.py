import time
import math
import io

#import penguinPi as ppi
#import picamera
#import picamery.array

import cv2
import numpy as np

## current configuration as global variable
xCurrent = 0
yCurrent = 0
thetaCurrent = 0

def toPoint(xTarget, yTarget,vel):
    Kv = 1
    Kh = 1
    thetaTarget = math.atan2((yTarget - yCurrent)/(xTarget - xCurrent))
    velAv = Kv * sqrt((xTarget-xCurrent)**2 + (yTarget-yCurrent)**2)        #offset due to min robot speed
    velDiff = Kh * (thetaTarget - thetaCurrent)
