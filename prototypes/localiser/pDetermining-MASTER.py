import time
import math
import io

import penguinPi as ppi
#import picamera
#import picamera.array

import cv2
import numpy as np
#import matplotlib.pyplot as plt

ppi.init()
mA = ppi.Motor(ppi.AD_MOTOR_A)
mB = ppi.Motor(ppi.AD_MOTOR_B)

## conversion from/to raw data
wheel_dia = 0.065
turn_dia = 0.145
p = 45
motor_L = -p
motor_R = -p
error = 4

##### image processing stuff #####
IMAGE_WIDTH = 320
IMAGE_HEIGHT=240

##### camera stuff #####

IMAGE_WIDTH = 320
IMAGE_HEIGHT=240
#camera = picamera.PiCamera()
#camera.framerate = 3
#camera.resolution = (IMAGE_WIDTH, IMAGE_HEIGHT)
#rawCapture = picamera.array.PiRGBArray(camera, size=(IMAGE_WIDTH, IMAGE_HEIGHT))

## goal stuff
#lower_yellow = np.array([20,100,100])
#upper_yellow = np.array([40,255,255])

#params_goaly = cv2.SimpleBlobDetector_Params()
#params_goaly.minThreshold = 10
#params_goaly.maxThreshold = 255
#params_goaly.filterByColor = False
#params_goaly.blobColor = 255
#params_goaly.filterByArea = True
#params_goaly.minArea = 0
#params_goaly.maxArea = 30000
#params_goaly.filterByCircularity = False
#params_goaly.minCircularity = 0
#params_goaly.filterByInertia = False
#params_goaly.minInertiaRatio = 0.05
#params_goaly.filterByConvexity = False
#params_goaly.minConvexity = 0.87
#detector_goaly = cv2.SimpleBlobDetector_create(params_goaly)



def gamma_correction(img, correction):
    img = img/255.0
    img = cv2.pow(img, correction)
    return np.uint8(img*255)

stream = io.BytesIO()
def capture_image1():
    camera.capture(stream, format='jpeg', use_video_port=True)
    data = cv2.imdecode(np.fromstring(stream.getvalue(), dtype=np.uint8), 1)
    return data


def getGoalPosPixel(img):#(image_path):

    # detect yellow blob
    #img = cv2.imread(image_path,1)
    #img = np.asarray(data, dtype=np.uint8)
    gamma_conv = gamma_correction(img,2.2)
    convHSV = cv2.cvtColor(gamma_conv, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(convHSV,lower_yellow,upper_yellow)
    masked = cv2.bitwise_and(convHSV,convHSV,mask=mask)

    #cv2.imwrite('1.png',mask)
    mask = cv2.dilate(mask,np.ones((5,5),np.uint8),iterations = 1)
    #cv2.imshow('mask',mask)

    keypoints = detector_goaly.detect(mask)

    obj_pix = keypoints[0].size * 0.0000001583235294117647
    print(keypoints[0].size)
    focal = 3.04/1000
    obj_real = 0.09

    const = 0.0026046511
    toTarget_x = obj_real*focal/obj_pix
    print(keypoints[0].pt[0])
    toTarget_y = -(keypoints[0].pt[0]-IMAGE_WIDTH/2)*(toTarget_x * const)

    # return yellow blob, if some detected
    if keypoints != []:
        return np.floor(keypoints[0].pt).astype(int), toTarget_x/100, toTarget_y/100


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

## motion model of lab robot
def get_motion_sim(v,w,delta_t):
    #input
    # v: linear velocity (m/s)
    # w: angular velocity (m/s)
    # delta_t: time interval
    #output
    # delta_d: distance travelled
    # delta_th: change in heading
    delta_d = v * delta_t + 0.05 * np.random.randn(1)[0]
    delta_th = w * delta_t + 0.01 * (np.pi/180) * np.random.randn(1)[0]

    return delta_d,delta_th

def get_motion(tL2,tL1,tR2,tR1):
    diffL = -enc_diff(tL2,tL1)
    diffR = -enc_diff(tR2,tR1)
    print('Diff L:'+str(diffL))
    print('Diff R:'+str(diffR))
    delta_d = wheel_dia*np.pi*(diffL + diffR)/(2*360)
    delta_theta = ((diffR - diffL)/180)*(wheel_dia/turn_dia)    #TODO: verify!

    return delta_d,delta_theta

## current configuration as global variable
xCurrent = 0
yCurrent = 0
thetaCurrent = 0

def toPoint(xTarget, yTarget, xCurrent, yCurrent, thetaCurrent):
    
    #plt.axis([-1.5,1.5,-1.5,1.5])
    #plt.ion()
    #plt.plot(xTarget,-yTarget,marker='*',color='r')
    
    ## control params
    Kv = 0.7 #0.7 
    Kh = 1.0 #0.5
    goal_tolerance = 0.1

    #TODO: loop until target is within a tolerance
    t1 = t2 = v = w = 0.0
    tL1 = mA.get_ticks()
    tR1 = mB.get_ticks()

    while(True):
        ## calculate current configuration
        t2 = time.time(); tL2 = mA.get_ticks(); tR2 = mB.get_ticks()
        #delta_d, delta_th = get_motion_sim(v,w,t2-t1) ## For simulation
        delta_d, delta_th = get_motion(tL2,tL1,tR2,tR1) ## On Robot
    
        xCurrent = xCurrent + delta_d * math.cos(thetaCurrent)
        yCurrent = yCurrent + delta_d * math.sin(thetaCurrent)
        thetaCurrent = thetaCurrent + delta_th
        delta_t = t2 - t1
        t1 = t2 ; tL1 = tL2; tR1 = tR2


        ## break if goal is reached
        if ((xCurrent-xTarget)**2 + (yCurrent-yTarget)**2 < goal_tolerance**2):
            break

        ## angle to target (not pose angle!)
        thetaTarget = math.atan2((yTarget - yCurrent),(xTarget - xCurrent))

        ## calculate desired motion speeds
        velAv = Kv * math.sqrt((xTarget-xCurrent)**2 + (yTarget-yCurrent)**2)        #offset due to min robot speed
        velDiff = Kh * (thetaTarget - thetaCurrent)
        vL = velAv - velDiff/2
        vR = velAv + velDiff/2
        #print(vL)
        #print(vR)
        if vL > 0.5:
            vL = 0.5
        if vR > 0.5:
            vR = 0.5
        if vL < -0.5:
            vL = -0.5
        if vR< -0.5:
            vR = -0.5

        #print(thetaTarget)
        #print(thetaCurrent)
               
        ## set motor settings
        mA.set_power(speed2powerLeft(vL))
        mB.set_power(speed2powerRight(vR))
        
        #arrow_size = 0.05
        #dx = arrow_size * math.cos(thetaCurrent)
        #dy = arrow_size * math.sin(thetaCurrent)
        #plt.plot([xCurrent,xCurrent+dx],[-yCurrent,-(yCurrent+dy)],color='r',linewidth=3)
        #plt.plot([xCurrent,xCurrent-dy],[-yCurrent,-(yCurrent+dx)],color='b',linewidth=3)
        
        time.sleep(0.1)
        print('---------')
        print('vL = '+str(vL))
        print('vR = '+str(vR))
        print('x = ' + str(xCurrent))
        print('y = ' + str(yCurrent))
        print('delta_d = ' + str(delta_d))
        print('delta_th = ' + str(delta_th))
        #print(thetaCurrent)
        #print(delta_th)
    print('done')   
    mA.set_power(0)
    mB.set_power(0)
    
    plt.show()
    input()
    
    return xCurrent, yCurrent, thetaCurrent

if __name__ == '__main__':
    #x = 1
    #y = 1
    #xCurrent, yCurrent, thetaCurrent = toPoint(x,-y, xCurrent, yCurrent, thetaCurrent)
    #img = capture_image1()
    #key,x,y = getGoalPosPixel(img)
    #print('---------')
    #print('x = '+str(x))
    #print('y = '+str(y))
    xCurrent, yCurrent, thetaCurrent = toPoint(.9,-0, xCurrent, yCurrent, thetaCurrent)

