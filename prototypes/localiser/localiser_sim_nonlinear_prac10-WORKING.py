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
from numpy.linalg import inv
import numpy.random as rnd
from matplotlib.patches import Ellipse

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
xT = np.transpose(data['xr'])
xT = xT[1:np.size(xT),:]
sensor = data['sensor']
num_z = mp.shape[0]

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

def plotEllipse(mu,sigma,std):
    ax=plt.gca()
    sigma = sigma[:2,:2]
    #sigma = inv(sigma)
    eig = np.linalg.eig(sigma)
    #print('sigma:' + str(sigma))
    print('eig:' + str(eig))
    W = std*math.sqrt(eig[0][0])*4
    H = std*math.sqrt(eig[0][1])*4
    A = np.rad2deg(np.arctan2(eig[1][1,0],eig[1][0,0]))
    ellipse = Ellipse(xy=(mu[0,0],mu[1,0]), width=W, height=H,angle=A,fill=False)
    ax.add_patch(ellipse)
    return 0

 
def wraptopi(x):
        pi = np.pi
        x = x - np.floor(x/(2*pi)) *2 *pi
        if (x>=pi):
            result = x - 2*pi
        else:
            result = x 
        return result

def ask_the_oracle(xT,k):
    ## x is the pose of the robot at time step k
    xTrue = np.transpose(xT[k,:])
    return xTrue 

def get_odom(odom, k):
    delta_d = odom[k,0]
    delta_th = odom[k,1]
    return delta_d, delta_th

def get_map(mp):
    return mp

def sense(sensor,num_z,k):
    ## Get sensor data at time step k
    idx = (k)*num_z
    z = sensor[idx:idx+num_z,:] # NEED TO: check that it is indexing correctly. 
    Z = np.transpose(z) ## 2x1 matrix
    return Z   

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
    
          
    
    ## Kalman - coefs
     ## R
    sigmaT = 0.3 #1 ## 0.001
    sigmaR = wraptopi(np.deg2rad(15))
    ## Q
    sigmaRang = 0.1 ## 0.5 
    sigmaB = wraptopi(np.deg2rad(10))
     
    deltaT = 1
    
    R = np.matrix([[sigmaT**2, 0],[0,sigmaR**2]])
    Q = np.matrix([[sigmaRang**2, 0],[0, sigmaB**2]])
    
    ## Kalman - initiliaze
    X = np.matrix([[0],[0],[0]])
    cov = np.eye(3) ## 3x3
    I = np.eye(3) ## 3x3
    
    plt.figure()
    plt.axis([-100,100,-100,100])
    for k in range(0, 49):
    ## Data given
        ## Get odometry data
        delta_d, delta_theta = get_odom(odom, k)     
        ## Get sensor data
        Z = sense(sensor, num_z, k) ## True range and bearing to landmarks. 2x1
    ## Kalman - start
      ## Predicts
        ## Predict X
        X = X + np.matrix([[delta_d*wraptopi(math.cos(X[2,0]))],[delta_d*wraptopi(math.sin(X[2,0]))],[wraptopi(delta_theta)]]) ## 3x1
        ## Calculate Jx
        Jx = np.matrix([[1,0,-delta_d*wraptopi(math.sin(X[2,0]))],[0,1,delta_d*wraptopi(math.sin(X[2,0]))],[0,0,1]]) ## 3x2
        ## Calculate Ju
        Ju = np.matrix([[wraptopi(math.cos(X[2,0])),0],[wraptopi(math.sin(X[2,0])),0],[0,1]]) ## 3x3
        ## Predict cov
        cov = Jx*cov*np.transpose(Jx)+Ju*R*np.transpose(Ju) 
        print('Ju:'+str(Ju))
        print('Jx'+str(Jx))
        print('PredictCov:'+str(cov))
      
      ## Update
        for i in range(0,5):
            #print('X:'+str(X))
            r = math.sqrt(((mp[i,0]-X[0,0])**2) + ((mp[i,1]-X[1,0])**2))
            ## Calculate G
            G = np.matrix([[-(mp[i,0]-X[0,0])/r, -(mp[i,1]-X[1,0])/r, 0 ],[(mp[i,1]-X[1,0])/(r**2), -(mp[i,0]-X[0,0])/(r**2),-1]]) ## 2x3
            #G = np.matrix([[-(mp[i,0]*wraptopi(math.cos(mp[i,1]))-X[0,0])/mp[i,0], -(mp[i,0]*wraptopi(math.sin(mp[i,1]))-X[1,0])/mp[i,0], 0 ],[(mp[i,0]*wraptopi(math.sin(mp[i,1]))-X[1,0])/(mp[i,0]**2), -(mp[i,0]*wraptopi(math.cos(mp[i,1]))-X[0,0])/(mp[i,0]**2),-1]]) ## 2x3
            ## Calculate K
            K = cov*np.transpose(G)*inv(G*cov*np.transpose(G)+Q) ## 3x2
            ## Calculate H
            H = np.matrix([[math.sqrt((X[0,0]-mp[i,0])**2+(X[1,0]-mp[i,1])**2)],[wraptopi(math.atan2(mp[i,1]-X[1,0],mp[i,0]-X[0,0])-X[2,0])]])  ## 2x1 ## TODO: May not be calculated correctly
            ## Update X 
            error = (np.transpose(np.asmatrix(Z[:,i])) - H)
            #print('Z:'+str(Z[:,i]))
            #print('H:'+str(H))
            #print('error:'+str(error))
            dx = K*error
            #print('K*error'+str(dx))
            X = X + K*error
            #print(H)
            #print(np.transpose(np.asmatrix(Z[:,i])))
            ## Update cov 
            cov = (I-K*G)*cov
            #print('K:'+str(K))
            print('UpdateCov'+str(cov))
        print('--------------------')
       ## Standard deviation size
        std = 3 
        plotEllipse(X,cov,std)
      
    ## Kalman - finish
        
        xTrue = ask_the_oracle(xT,k)
        #print('--------------')
        #print('True pos: '+ str(xTrue[0])+','+str(xTrue[1]))
        #print('Estimate pos: '+ str(X))
        #print('Estimate pos: '+ str(X[0,0])+','+str(X[1,0]))
        #print('Kalman Gain:'+str(K))
        #print('Kalman Gain:'+str(K))
        ## plot x_true
        #arrow_size = 0.05
        #dx = arrow_size*math.cos(x_true[0])
        #dy = arrow_size*math.sin(x_true[1])
        plt.plot(xTrue[0],xTrue[1],'o',color='g')
        #plt.plot(X[0,0],X[1,0],'x',color='r')
        plt.plot(X[0,0],X[1,0],marker=(3, 0, np.rad2deg(X[2,0])), markersize=8,color='r',linestyle='None')

        #print(bF)
        #plt.plot(bF[0,0],bF[1,0],'x',color='b')
        #plt.plot([pEst[[0]],pEst[[0]]+0.01],[pEst[[1]],pEst[[1]]+0.01],color='b',linewidth=3)
    plt.plot(mp[:,0],mp[:,1],'x',color='y')
    #plt.plot(pEstMatrix[:,0], pEstMatrix[:,1],'x',color='b')
    #plt.plot(XEst[:,0], XEst[:,1],'x',color='b')
    plt.show()
    #input()
    #print(x_true)
    #print(pEst)    
    print('done')
