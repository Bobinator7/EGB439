import time
import math
import io

#import penguinPi as ppi
#import picamera
#import picamery.array

import cv2
import numpy as np
import matplotlib.pyplot as plt
import scipy.io as sio

## conversion from/to raw data
wheel_dia = 0.065
turn_dia = 0.145
p = 45
motor_L = -p
motor_R = -p
error = 4

## import simulation data.mat
data = sio.loadmat('data.mat')
mp = data['map']
odom = data['odom']
X = np.transpose(data['xr'])
X = X[2:np.size(X),:]
sensor = data['sensor']
num_z = np.size(mp)

## import goals, beacons, starting position
## [id, x, y, 0]
items = sio.loadmat('map.mat')
robotInitPos = items['map'][0]
goal1 = items['map'][1]
goal2 = items['map'][2]
beacon27 = items['map'][3]
beacon29 = items['map'][4]
beacon38 = items['map'][5]
beacon45 = items['map'][6]
beacon57 = items['map'][7]

def ask_the_oracle(X,k):
    ## x is the pose of the robot at time step k
    x = np.transpose(X[k,:])
    return x 

def get_odom(odom, k):
    delta_d = odom[k,0]
    delta_th = odom[k,1]
    return delta_d, delta_th

def get_map(mp):
    return mp

def sense(sensor,num_z,k):
    ## Get sensor data at time step k
    idx = 1 + (k-1)*num_z
    z = sensor[idx:idx+num_z-1,:] # NEED TO: check that it is indexing correctly.
    return z   

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
    diffL = enc_diff(tL2,tL1)
    diffR = enc_diff(tR2,tR1)
    delta_d = (diffL + diffR)/2
    delta_theta = ((diffR - diffL)/360)*(wheel_dia/turn_dia)    #TODO: verify!

    return delta_d,delta_th

## current configuration as global variable
xCurrent = 0
yCurrent = 0
thetaCurrent = 0

def localiser(delta_t):
    data = sio.loadmat('data.mat')
    A = np.array([[1,0,delta_t,0],[0,1,0,delta_t],[0,0,1,0],[0,0,0,1]])
    B = np.array([[(delta_t**2)/2, 0],[0,(delta_t**2)/2],[delta_t,0],[0,delta_t]])
    return

def toPoint(xTarget, yTarget, xCurrent, yCurrent, thetaCurrent):
    
    plt.axis([0,4,0,4])
    plt.ion()
    
    ## control params
    Kv = 0.7
    Kh = 2
    goal_tolerance = 0.01

    #TODO: loop until target is within a tolerance
    t1 = t2 = v = w = 0.0

    while(True):
        ## calculate current configuration
        t2 = time.time(); #tL2 = mA.get_ticks(); tR2 = mB.get_ticks()
        delta_d, delta_th = get_motion_sim(v,w,t2-t1) ## For simulation
        #delta_d, delta_th = get_motion(tL2,tL1,tR2,tR1) ## On Robot
    
        xCurrent = xCurrent + delta_d * math.cos(thetaCurrent)
        yCurrent = yCurrent + delta_d * math.sin(thetaCurrent)
        thetaCurrent = thetaCurrent + delta_th
        thetaCurrent = thetaCurrent + delta_th
        delta_t = t2 - t1
        t1 = t2 ; #tL1 = mA.get_ticks(); tR1 = mB.get_ticks()


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
        
        arrow_size = 0.05
        dx = arrow_size * math.cos(thetaCurrent)
        dy = arrow_size * math.sin(thetaCurrent)
        #plt.arrow(xCurrent,yCurrent,dx,dy,width=0.003)
        plt.plot([xCurrent,xCurrent+dx],[yCurrent,yCurrent+dy],color='r',linewidth=3)
        plt.plot([xCurrent,xCurrent-dy],[yCurrent,yCurrent+dx],color='b',linewidth=3)
        plt.plot(goal1[1],goal1[2],color='g',linewidth=3)
    
        plt.plot([3.25,3.25],[3.75,3.75],color='g',linewidth=3)
        time.sleep(0.1)
        #print(xCurrent)
        #print(yCurrent)
        
    plt.show()
    input()
    
    return xCurrent, yCurrent, thetaCurrent



if __name__ == '__main__':
    
    fig=plt.figure()
        
    #xCurrent, yCurrent, thetaCurrent = toPoint(1,2, xCurrent, yCurrent, thetaCurrent)
    ## Initialize variables
    Ez = 0.01 ## Measurement error
    C = np.array([1, 1, 1, 1])
    pEstMatrix = np.zeros((50,4))
    thetaPast = xPast = yPast = vxPast = vyPast = 0
    thetaCurrent = xCurrent = yCurrent = 0
    ## Kalman - coefs
    sigmaW = 0.5
    sigmaV = 0.001
    sigmaY = sigmaX = 0.1
    sigmaZ = 0.01
    propXY = 1
    deltaT = 1
    ## Kalman - matricies
    A = np.array([1 0 deltaT 0],[0 1 0 deltaT],[0 0 1 0],[0 0 0 1])
    B = np.array([(deltaT**2)/2 0],[0 (deltaT**2)/2],[deltaT 0],[0 deltaT]) 
    H = np.array([1 0 0 0],[0 1 0 0])
    R = np.array([sigmaV 0 0 0],[0 sigmaV 0 0],[0 0 sigmaV**2],[0 0 0 sigmaV**2])
    Q = np.array([sigmaW**2 0],[0 sigmaW**2])
    ## Kalman - initiliaze
    x = 0; y = 0; vx = 0; vy = 0; theta = 0
    cov = np.array([sigmaX**2 propXY*sigmaX*sigmaY],[propXY*sigmaX*sigmaY sigmaY**2])
    X = np.array([0],[0],[0],[0])
    I = np.eye(4)


    for k in range(0, 49):
    ## Data given
        delta_d, delta_theta = get_odom(odom, k)     
        Z = sense(sensor, num_z, k)

        U = np.array([x+delta_d*math.cos(delta_theta+theta)],[y+delta_d*math.sine(delta_theta+theta)],[delta_d*math.cos(delta_theta+theta)/deltaT],[delta_d*math.sine(delta_theta+theta)])
    ## Kalman - start
        ## Predict
        X = A*X + B*U      
        cov = A*cov*np.transpose(A)+R
        ## Update
        K = cov*np.transpose(H)/(H*cov*np.tranpose(H)+Q)
        X = X + K(Z - H*X) ## TODO: convert Z from range, bearing to x,y of robots position
        cov = (I-K*H)*cov
    ## Kalman - finnish
   
        thetaCurrent = thetaCurrent + delta_theta
        xCurrent = xCurrent + delta_d*math.cos(thetaCurrent)
        yCurrent = yCurrent + delta_d*math.sin(thetaCurrent)
        
        ## Assume delta_t (time step) to be 1. 
        controlInput = np.array([xPast+vxPast,yPast+vyPast,xCurrent-xPast,yCurrent-yPast])
        pPastEst = np.array([xPast,yPast,vxPast,vyPast])
        
        pEst = pPastEst + controlInput
        
        ## For plotting
        pEstMatrix[k,:] = pEst[:]
        
        ## Pass current values to past variables
        thetaPast = thetaCurrent
        xPast = xCurrent
        yPast = yCurrent
        
        K = pEst*C/(C*pEst*np.transpose(C)+Ez)
        #pUpdate = pEst + K()
        x_true = ask_the_oracle(X,k)
        print('--------------')
        print('True pos: '+ str(x_true[0])+','+str(x_true[1]))
        print('Estimate pos: '+ str(pEst[0])+','+str(pEst[1]))
        print('Kalman Gain:'+str(K))
        ## plot x_true
        arrow_size = 0.05
        dx = arrow_size*math.cos(x_true[0])
        dy = arrow_size*math.sin(x_true[1])
        plt.plot([x_true[0],x_true[0]+dx],[x_true[1],x_true[1]+dy],color='g',linewidth=3)
        
        #plt.plot([pEst[[0]],pEst[[0]]+0.01],[pEst[[1]],pEst[[1]]+0.01],color='b',linewidth=3)
    plt.plot(mp[:,0],mp[:,1],'x',color='r')
    plt.plot(pEstMatrix[:,0], pEstMatrix[:,1],'x',color='b')
    plt.show()
    input()
    #print(x_true)
    #print(pEst)    
    print('done')
