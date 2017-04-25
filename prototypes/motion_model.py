import time
import math
import io

#import penguinPi as ppi
#import picamera
#import picamery.array

import cv2
import numpy as np
import matplotlib.pyplot as plt

## conversion from/to raw data
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

def get_motion(tL1,tL2,tR1,tR2):
    diffL = enc_diff(tL1,tL2)
    diffR = enc_diff(tR1,tR2)
    delta_d = (diffL + diffR)/2
    delta_theta = ((diffR - diffL)/720)*(wheel_dia/turn_dia)    #TODO: verify!

    return delta_d,delta_th

## current configuration as global variable
xCurrent = 0
yCurrent = 0
thetaCurrent = 0

def toPoint(xTarget, yTarget, xCurrent, yCurrent, thetaCurrent):
    
    plt.axis([-5,5,-5,5])
    plt.ion()
    
    ## control params
    Kv = 0.5
    Kh = 1.1
    goal_tolerance = 0.01

    #TODO: loop until target is within a tolerance
    t1 = t2 = v = w = 0.0

    while(True):
        ## calculate current configuration
        t2 = time.time()
        delta_d, delta_th = get_motion_sim(v,w,t2-t1)
        xCurrent = xCurrent + delta_d * math.cos(thetaCurrent)
        yCurrent = yCurrent + delta_d * math.sin(thetaCurrent)
        thetaCurrent = thetaCurrent + delta_th
        t1 = t2


        ## break if goal is reached
        if ((xCurrent-xTarget)**2 + (yCurrent-yTarget)**2 < goal_tolerance):
            break

        ## angle to target (not pose angle!)
        thetaTarget = math.atan2((yTarget - yCurrent),(xTarget - xCurrent))

        ## calculate desired motion speeds
        velAv = Kv * math.sqrt((xTarget-xCurrent)**2 + (yTarget-yCurrent)**2)        #offset due to min robot speed
        velDiff = Kh * (thetaTarget - thetaCurrent)
        vL = velAv - velDiff/2
        vR = velAv + velDiff/2
        
        print(thetaTarget)
        print(thetaCurrent)

        ## set motor settings
        #mA.set_power(speed2powerLeft(vL))
        #mB.set_power(speed2powerRight(vR))

        v, w = velAv, velDiff       #only for simulation
        
        plt.scatter(xCurrent,yCurrent)
        plt.show()

        time.sleep(0.1)
        #print(xCurrent)
        #print(yCurrent)

if __name__ == '__main__':
    
    fig=plt.figure()
        
    toPoint(2,3, xCurrent, yCurrent, thetaCurrent)
    
    
    input()
    
