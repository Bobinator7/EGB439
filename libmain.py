###
# Author: Johnson Loh, Callum Tyler
# Date: 18 April 2017
# Subject: EGB439 (SLAM Project)

###
# movement:
# speed2powerLeft(v)
# speed2powerRight(v)
# enc_diff(init_count, current_count)
# translate(distance, vel)
# rotate(angle, motor_L, motor_R) #TODO: single motor power setting

###
# image processing:
# gamma_correction(img, correction)
# getGoalPosPixel(img)

##### libraries #####
import time
import math 
import io

import penguinPi as ppi
import picamera
import picamera.array

import cv2
import numpy as np

##### initialize penguinPi #####
ppi.init()
mA = ppi.Motor(ppi.AD_MOTOR_A)
mB = ppi.Motor(ppi.AD_MOTOR_B)

##### movement API #####
wheel_dia = 0.065
turn_dia = 0.145
p = 45
motor_L = -p
motor_R = -p
error = 4

def speed2powerLeft(v):
    power = round(v * (-122.00) - 16.75) #-113.6363 -16.75
    return power

def speed2powerRight(v):
    power = round(v * (-116.6512) - 16.65)
    return power

def enc_diff(init_count, current_count):
    half_res = 15360
    full_res = 30720
    scaled_count = current_count - init_count + half_res
    return_count = current_count - init_count
    if (scaled_count < 0):
        return_count = (half_res - init_count) + (current_count + half_res)
    elif (scaled_count >= full_res):
        return_count = (half_res - current_count) + (init_count + half_res) 
    return return_count

def toPoint(xTarget, yTarget, vel):
    Kv = 1
    Kh = 1
    xCurrent = 0
    yCurrent = 0
    thetaCurrent = 0 
    thetaTarget = math.atan((yTarget - yCurrent)/(xTarget - xCurrent))
    velAv = Kv*sqrt((xTarget-xCurrent)**2 + (yTarget-yCurrent)**2)
    velDiff = Kh*(thetaTarget - thetaCurrent)
     

def translate(distance, vel):
    motor_L = speed2powerLeft(vel)
    motor_R = speed2powerRight(vel)
    
    ticks_translate = distance*(360/(3.14*wheel_dia))
    tick_diffT = 0
    countA_tot = 0
    countB_tot = 0
    epsilon = 40
    limit = 2
    increment = 0.25

    t1A = mA.get_ticks()
    t1B = mA.get_ticks()
    while tick_diffT < ticks_translate:
        mA.set_power(motor_L)
        mB.set_power(motor_R)
        t2A = mA.get_ticks()
        t2B = mA.get_ticks()
        tick_diffT = abs(t2A - t1A)
        countA = enc_diff(t1A, t2A)
        countB = enc_diff(t1B, t2B)

        countA_tot = countA_tot + countA
        countB_tot = countB_tot + countB

    if (abs(countA_tot - countB_tot) > epsilon):
        if (countA_tot < countB_tot):
            if ((motor_L - motor_R) < limit ):
                motor_L = round(motor_L + increment)
        elif (countA_tot > countB_tot):
            if ((motor_R - motor_L) < limit) :
                motor_L = round( motor_L - increment)

    mA.set_power(0)
    mB.set_power(0)

    return

def rotate(angle, motor_L, motor_R):
    ticks_rotate = (turn_dia*angle/wheel_dia) - error
    tick_diff = tick_diffL = tick_diffR = 0 
    t3L = mA.get_ticks()
    t3R = mB.get_ticks()
    while tick_diff < ticks_rotate:
        mA.set_power(motor_L)
        mB.set_power(-motor_R)
        t4L = mA.get_ticks()
        t4R = mB.get_ticks()
        tick_diffL = enc_diff(t3L, t4L)
        tick_diffR = enc_diff(t3R, t4R)
        tick_diff = (abs(tick_diffL) + abs(tick_diffR))/2

    # mA.set_power(-100)
    # mB.set_power(100)
    # time.sleep(0.05)	
    mA.set_power(0)
    mB.set_power(0)
    return

##### camera setting #####

IMAGE_WIDTH = 320   
IMAGE_HEIGHT=240
camera = picamera.PiCamera()
camera.framerate = 3 
camera.resolution = (IMAGE_WIDTH, IMAGE_HEIGHT)
rawCapture = picamera.array.PiRGBArray(camera, size=(IMAGE_WIDTH, IMAGE_HEIGHT))

## camera loop skeleton
#for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
#
#    image = np.uint8(frame.array)
#    # process image
#    rawCapture.truncate(0) 


##### image processing API #####

## gamma correction 
def gamma_correction(img, correction):
    img = img/255.0
    img = cv2.pow(img, correction)
    return np.uint8(img*255)

## detect yellow goal 
lower_yellow = np.array([20,100,100])
upper_yellow = np.array([40,255,255])

params_goaly = cv2.SimpleBlobDetector_Params()
params_goaly.minThreshold = 10
params_goaly.maxThreshold = 255
params_goaly.filterByColor = False
params_goaly.blobColor = 255
params_goaly.filterByArea = True
params_goaly.minArea = 0
params_goaly.maxArea = 30000
params_goaly.filterByCircularity = False 
params_goaly.minCircularity = 0
params_goaly.filterByInertia = False
#params_goaly.minInertiaRatio = 0.05
params_goaly.filterByConvexity = False
#params_goaly.minConvexity = 0.87
detector_goaly = cv2.SimpleBlobDetector_create(params_goaly)

def getGoalPosPixel(img):
    
    gamma_conv = gamma_correction(img,2.2)
    convHSV = cv2.cvtColor(gamma_conv, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(convHSV,lower_yellow,upper_yellow)
    masked = cv2.bitwise_and(convHSV,convHSV,mask=mask)

    mask = cv2.dilate(mask,np.ones((5,5),np.uint8),iterations = 1)

    keypoints = detector_goaly.detect(mask)
    
    # return yellow blob, if some detected
    if keypoints != []:
        #im_with_keypoints = cv2.drawKeypoints(masked[:,:,2], keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        #cv2.imshow('keypoints', im_with_keypoints)
        #cv2.waitKey(0)
        #print(keypoints[0].pt) 
        return np.floor(keypoints[0].pt).astype(int)

    # return empty list otherwise
    return []
