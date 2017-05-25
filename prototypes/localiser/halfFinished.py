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

## Camera settings
IMAGE_WIDTH = 320
IMAGE_HEIGHT=240
camera = picamera.PiCamera()
camera.framerate = 3
camera.resolution = (IMAGE_WIDTH, IMAGE_HEIGHT)
rawCapture = picamera.array.PiRGBArray(camera, size=(IMAGE_WIDTH, IMAGE_HEIGHT))

## Input image:
#img = cv2.imread('beaconImage337.jpg')

stream = io.BytesIO()
def capture_image1():
    camera.capture(stream, format='jpeg', use_video_port=True)
    data = cv2.imdecode(np.fromstring(stream.getvalue(), dtype=np.uint8), 1)
    return data

def getChrom(img):
    chrom = np.zeros((img.shape[0],img.shape[1],3), np.float32)
    lum = np.zeros((img.shape[0],img.shape[1]), np.uint16)
    img = img.astype(np.uint16)
    
    for i in range(0,img.shape[0]):
        for j in range(0,img.shape[1]):
            b = img[i,j,0]
            g = img[i,j,1]
            r = img[i,j,2]
            sum = b + g + r  
            chrom[i,j,0] = img[i,j,0] / sum
            chrom[i,j,1] = img[i,j,1] / sum
            chrom[i,j,2] = img[i,j,2] / sum
            lum[i,j] = sum
            
    return chrom, lum

def getGrayMask(img):
    mask = np.zeros((img.shape[0],img.shape[1]), np.uint8)
    
    for i in range(0,img.shape[0]):
        for j in range(0,img.shape[1]):
            minimum = 255
            maximum = 0
            for val in range(0,3):
                if(img[i,j,val] < minimum):
                    minimum = img[i,j,val]
                if(img[i,j,val] > maximum):
                    maximum = img[i,j,val]
                    
            #print(maximum)
            #print(minimum)
                    
            if (maximum-minimum < 0.05):
                #print(maximum)
                #print(minimum)
                #print(img[i,j,:])
                mask[i,j] = 255
                
    return mask

def chunks(l, n):
    """Yield successive n-sized chunks from l."""
    tolerance = 3
    
    i = 0
    while i < len(l):
        current_x = 0
        for j in range(0,n):
            if i+j >= len(l):
                break

            if j == 0:
                current_x = l[i+j][0]
            else:
                if l[i+j][0] - current_x > tolerance:
                    break
                
        if i+j >= len(l):
            break        


        if l[i+j][0] - current_x > tolerance:
            i = i + j
            continue
        else:    
            yield l[i:i + n]
            i = i + n
            continue

def getBeaconID(img):
    img = cv2.blur(img,(5,5))
    cv2.imshow('orig',img)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    equ = cv2.equalizeHist(gray)
    
    chrom, lum = getChrom(img)
    #print(lum)
    #print(equ.dtype)
    
    m_black = lum < 70
    m_black = m_black * 255.
    m_black = m_black.astype(np.uint8)
    print(m_black.dtype)
    cv2.imshow('test2',m_black)
    
    
    #m_gray = getGrayMask(chrom)
    cv2.imshow('test',equ)
    
    
    
    ret,m_w = cv2.threshold(equ,150,255,cv2.THRESH_BINARY)
    
    ######
    ## color mask limits based on chromaticity values 
#    lower_r = np.array([0,0.08,0.57])
#    upper_r = np.array([1,0.29,0.9])
#    lower_g = np.array([0,0.35,0.35])
#    upper_g = np.array([1,0.51,0.63])
#    lower_b = np.array([0.30,0,0])
#    upper_b = np.array([0.9,1,1])
#    lower_fg = np.array([0,0.45,0.2])
#    upper_fg = np.array([1,0.56,0.32])
#    
#    ## create color masks
#    m_r = cv2.inRange(chrom,lower_r,upper_r)
#    m_g = cv2.inRange(chrom,lower_g,upper_g)
#    m_b = cv2.inRange(chrom,lower_b,upper_b)
#    m_fg = cv2.inRange(chrom,lower_fg,upper_fg)
    
    ######
    
    ####################
    ## color mask limits based on hsv values (gimp format: 0-360 0-100 0-100)
    lower_r1 = np.array([0,120,50])          #0 60 20
    upper_r1 = np.array([15,255,255])       #22 100 100
    lower_r2 = np.array([175,120,50])        #356 60 20
    upper_r2 = np.array([180,255,255])      #360 100 100
    lower_g = np.array([15,100,50])          #34 40 20
    upper_g = np.array([45,255,130])         #84 100 50
    lower_b = np.array([70,12,25])           #140 5 10
    upper_b = np.array([165,160,130])         #322 60 50
    lower_fg = np.array([45,112,25])         #90 45 10
    upper_fg = np.array([63,180,112])         #126 70 45
    #lower_w = np.array([0,0,120])
    #upper_w = np.array([180,255,255])

    ## create color masks (note: blue mask inaccurate -> verify blue beacon segment by red and green -> green less precise than red (verify?))
    m_r1 = cv2.inRange(hsv,lower_r1,upper_r1)
    m_r2 = cv2.inRange(hsv,lower_r2,upper_r2)
    m_r = m_r1 + m_r2
    m_g = cv2.inRange(hsv,lower_g,upper_g)
    m_b = cv2.inRange(hsv,lower_b,upper_b)
    m_fg = cv2.inRange(hsv,lower_fg,upper_fg)
    #m_w = cv2.inRange(hsv,lower_w,upper_w)
    ###################

    ## remove salt and pepper artifacts
    kernel_saltpepper = np.ones((5,5),np.uint8)
    filtered_m_r = cv2.morphologyEx(m_r, cv2.MORPH_OPEN, kernel_saltpepper)
    #filtered_m_r = cv2.morphologyEx(m_r, cv2.MORPH_CLOSE, kernel_saltpepper)
    filtered_m_g = cv2.morphologyEx(m_g, cv2.MORPH_OPEN, kernel_saltpepper)
    #filtered_m_g = cv2.morphologyEx(m_g, cv2.MORPH_CLOSE, kernel_saltpepper)
    #filtered_m_b = cv2.morphologyEx(m_b, cv2.MORPH_OPEN, kernel_saltpepper)
    filtered_m_b = cv2.morphologyEx(m_b, cv2.MORPH_CLOSE, kernel_saltpepper)
    #filtered_m_fg = cv2.morphologyEx(m_fg, cv2.MORPH_OPEN, kernel_saltpepper)
    filtered_m_fg = cv2.morphologyEx(m_fg, cv2.MORPH_CLOSE, kernel_saltpepper)
    
    kernel_dilate = np.ones((5,5),np.uint8) 
    #m_background = cv2.morphologyEx(m_gray, cv2.MORPH_OPEN, kernel_dilate) 
    #m_background = cv2.dilate(m_background,kernel_dilate,iterations = 1)
    m_background = cv2.bitwise_and(cv2.bitwise_not(m_w),cv2.bitwise_not(m_black))
    filtered_m_b = cv2.bitwise_and(m_background,filtered_m_b)
    filtered_m_g = cv2.bitwise_and(m_background,filtered_m_g)
    filtered_m_r = cv2.bitwise_and(m_background,filtered_m_r)
    
    ## debug filtered image
    #cv2.imshow('red',m_r)
    cv2.imshow('filtered red',filtered_m_r)
    #cv2.imshow('green',m_g)
    cv2.imshow('filtered green',filtered_m_g)
    cv2.imshow('blue',m_b)
    cv2.imshow('filtered blue',filtered_m_b)
    #cv2.imshow('floor',m_fg)
    #cv2.imshow('filtered floor green',filtered_m_fg)
    #cv2.imshow('white',m_w)
    cv2.imshow('back',m_background)

    ## get red and green blobs
    bbd = cv2.SimpleBlobDetector_Params()
    bbd.minThreshold = 10
    bbd.maxThreshold = 200
    bbd.filterByColor = True
    bbd.blobColor = 255
    bbd.filterByArea = True
    bbd.minArea = 10 
    bbd.maxArea = 1000
    bbd.filterByCircularity = True
    bbd.minCircularity = 0.5
    bbd.filterByInertia = False
    bbd.filterByConvexity = False
    detect_beacon = cv2.SimpleBlobDetector_create(bbd)
    
    br = detect_beacon.detect(filtered_m_r)
    bg = detect_beacon.detect(filtered_m_g)
    bb = detect_beacon.detect(filtered_m_b)

    test_r = cv2.drawKeypoints(img,br,np.array([]),(0,0,255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imshow('kp', test_r)
    test_g = cv2.drawKeypoints(img,bg,np.array([]),(0,0,255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imshow('kg', test_g)    
    test_b = cv2.drawKeypoints(img,bb,np.array([]),(0,0,255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imshow('kb', test_b)
    
    frame_tolerance = 10
    br = [i for i in br if int(round(i.pt[0])) >= frame_tolerance]
    br = [i for i in br if int(round(i.pt[1])) >= frame_tolerance]
    br = [i for i in br if int(round(i.pt[0])) < m_r.shape[1] - frame_tolerance]
    br = [i for i in br if int(round(i.pt[1])) < m_r.shape[0] - frame_tolerance]
    bg = [i for i in bg if int(round(i.pt[0])) >= frame_tolerance]
    bg = [i for i in bg if int(round(i.pt[1])) >= frame_tolerance]
    bg = [i for i in bg if int(round(i.pt[0])) < m_g.shape[1] - frame_tolerance]
    bg = [i for i in bg if int(round(i.pt[1])) < m_g.shape[0] - frame_tolerance]
    bb = [i for i in bb if int(round(i.pt[0])) >= frame_tolerance]
    bb = [i for i in bb if int(round(i.pt[1])) >= frame_tolerance]
    bb = [i for i in bb if int(round(i.pt[0])) < m_b.shape[1] - frame_tolerance]
    bb = [i for i in bb if int(round(i.pt[1])) < m_b.shape[0] - frame_tolerance]
   
    kp_list = []
    for it in br:
        kp_list.append([it.pt[0], it.pt[1], 1, it.size])
    for it in bg:
        kp_list.append([it.pt[0], it.pt[1], 2, it.size])
    for it in bb:
        kp_list.append([it.pt[0], it.pt[1], 3, it.size])
    kp_list.sort()
    
    print(kp_list)
    
    beacon_list = chunks(kp_list,3)
    result = []
    for it in beacon_list:
        #print(it)
        it.sort(key=lambda x: x[1])
#        print(it)
        ID = 0
        meanX = 0
        bigY = 0
        for m in range(0,3):
            ID = ID + it[m][2] * 2**((2-m)*2)
            #print(it[m][0])
            meanX = (meanX + it[m][0])
            if bigY < it[m][1]:
                bigY = it[m][1]
                
        meanX = meanX / 3
        beaconID = [meanX, bigY, ID, it[m][3]]
    
    return result.append(beaconID)

## TODO: Get Johnson to check this function
def xy2XY(keypoints):
    ## Assume that another function will ouput the keypoints of the beacons
    ## Transform xy of target in pixel to xy in robot world 
    obj_pix = keypoints[0,0].size * 0.0000001583235294117647
    print(keypoints[0].size)
    focal = 3.04/1000
    obj_real = 0.09

    const = 0.0026046511
    toTarget_x = obj_real*focal/obj_pix
    print(keypoints[0].pt[0])
    toTarget_y = -(keypoints[0].pt[0]-IMAGE_WIDTH/2)*(toTarget_x * const) ##TODO: Check that this is be properly indexed. 
    ## Not sure about below
    # return yellow blob, if some detected
    if keypoints != []:
        return np.floor(keypoints[0].pt).astype(int), toTarget_x/100, toTarget_y/100
    
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
    
    img = capture_image1()
    
    keypoints = getBeaconID(img)
             
    
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
            Range = math.sqrt((X[0,0]-mp[i,0])**2+(X[1,0]-mp[i,1])**2)
           
                         
            H = np.matrix([[Range],[wraptopi(math.atan2(mp[i,1]-X[1,0],mp[i,0]-X[0,0])-X[2,0])]])  ## 2x1 ## TODO: May not be calculated correctly
            ## Update X 
            error = (np.transpose(np.asmatrix(Z[:,i])) - H) ## TODO: if range to landmark in Z is above 1.5 (for example) ignore that particular beacon for localising.
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
