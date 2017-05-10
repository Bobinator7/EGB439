import time
import math
import io

import penguinPi as ppi
import picamera
import picamera.array

import cv2
import numpy as np
import matplotlib.pyplot as plt

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
camera = picamera.PiCamera()
camera.framerate = 3
camera.resolution = (IMAGE_WIDTH, IMAGE_HEIGHT)
rawCapture = picamera.array.PiRGBArray(camera, size=(IMAGE_WIDTH, IMAGE_HEIGHT))

## goal stuff
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



def gamma_correction(img, correction):
    img = img/255.0
    img = cv2.pow(img, correction)
    return np.uint8(img*255)

def capture_image():
    camera.capture(rawCapture, format="bgr")
    image_array = rawCapture.array
    rawCapture.truncate(0)
    return image_array

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
    count=0
    result = np.zeros([0,3])
    for it in beacon_list:

        it.sort(key=lambda x: x[1])

        ID = 0
        meanX = 0
        bigY = 0
        smallY = 900
        for m in range(0,3):
            # get ID
            ID = ID + it[m][2] * 2**((2-m)*2)
            
            # get horizontal position
            meanX = (meanX + it[m][0])
            
            # get size of beacon
            if smallY > it[m][1]:
                smallY = it[m][1]
            if bigY < it[m][1]:
                bigY = it[m][1]
                
        ## TODO convert meanX and bigY-smallY to range and bearing
        meanX = meanX / 3
        
        m1 = -0.0210896309314587
        b1 = 1.5588466973637962
        ran = m1*(bigY-smallY)+b1
        m2 = -0.142649154
        b2 = 1.530568848
        bearing = m2*((meanX-160)/r)+b2
        result = np.append(result,[[ID, ran, bearing ]],axis=0)
        count = count + 1
    
    return result


######## movement stuff ##############
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

def get_motion(tR2,tR1,tL2,tL1):
    diffL = -enc_diff(tL2,tL1)
    diffR = -enc_diff(tR2,tR1)
    print('Diff L:'+str(diffL))
    print('Diff R:'+str(diffR))
    delta_d = wheel_dia*np.pi*(diffL + diffR)/(2*360)
    delta_theta = ((diffR - diffL)/180)*(wheel_dia/turn_dia)

    return delta_d,delta_theta

###### utility #########
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

## current configuration as global variable
robot = [0.2,0.2]
goal1 = [0.635, 1.350]
goal2 = [1.44, 1.290]
xCurrent = robot[0]
yCurrent = robot[1]
thetaCurrent = 0

map_static = np.array([[27,1.060,0.125],[29,1.775,0.695],[38,1.790,1.745],[45,0.975,1.790],[57,0.180,1.380]])

def toPoint(xTarget, yTarget, xCurrent, yCurrent, thetaCurrent):
    
    #plt.axis([-1.5,1.5,-1.5,1.5])
    
    ## motion control params
    Kv = 0.7 #0.7 
    Kh = 1.0 #0.5
    goal_tolerance = 0.1

    ## initialize values
    t1 = t2 = v = w = 0.0
    tR1 = mA.get_ticks(); tL1 = mB.get_ticks()
    tR2 = mA.get_ticks(); tL2 = mB.get_ticks()

    ## Kalman init
    sigmaT = 0.3
    sigmaR = wraptopi(np.deg2rad(15))
    R = np.matrix([[sigmaT**2, 0],[0,sigmaR**2]])
    
    sigmaRang = 0.1 
    sigmaB = wraptopi(np.deg2rad(10))
    Q = np.matrix([[sigmaRang**2, 0],[0, sigmaB**2]])
     
    deltaT = 1
    
    X = np.matrix([[xCurrent],[yCurrent],[thetaCurrent]])
    cov = np.eye(3)
    I = np.eye(3)
    

    while(True):
                
        ## get motion from encoders
        delta_d, delta_th = get_motion(tL2,tL1,tR2,tR1)
        
        ## get sensor
        img = capture_image1()
        Z = getBeaconID(img)
   
        #### KALMAN FILTER (start) ####
        ## predict step
        X = X + np.matrix([[delta_d*wraptopi(math.cos(X[2,0]))],[delta_d*wraptopi(math.sin(X[2,0]))],[wraptopi(delta_theta)]])
        Jx = np.matrix([[1,0,-delta_d*wraptopi(math.sin(X[2,0]))],[0,1,delta_d*wraptopi(math.sin(X[2,0]))],[0,0,1]])
        Ju = np.matrix([[wraptopi(math.cos(X[2,0])),0],[wraptopi(math.sin(X[2,0])),0],[0,1]])
        cov = Jx*cov*np.transpose(Jx)+Ju*R*np.transpose(Ju)
        
        ## update step
        mp = []
        for i in range(0,np.size(Z,0))
            for j in range(0,np.size(map_static,0)):
                if Z[i,0] == map_static[j,0]:
                    mp = map_static[j,1:]
                    break
                
            if mp == []:
                print('no beacon..')
                break
            
            r = math.sqrt(((mp[0]-X[0,0])**2) + ((mp[1]-X[1,0])**2))
            G = np.matrix([[-(mp[0]-X[0,0])/r, -(mp[1]-X[1,0])/r, 0 ],[(mp[1]-X[1,0])/(r**2), -(mp[0]-X[0,0])/(r**2),-1]])
            K = cov*np.transpose(G)*inv(G*cov*np.transpose(G)+Q)
            H = np.matrix([[r],[wraptopi(math.atan2(mp[1]-X[1,0],mp[0]-X[0,0])-X[2,0])]])
            
            error = (np.transpose(np.asmatrix(Z[i,1:])) - H) # TODO check if correct!
            X = X + K*error
            cov = (I-K*G)*cov
            
        ## plot covariance
        #std = 3 
        #plotEllipse(X,cov,std)
        
        #### KALMAN FILTER (end) ####

        ## break if goal is reached
        if ((X[0,0]-xTarget)**2 + (X[1,0]-yTarget)**2 < goal_tolerance**2):
            break

        ## angle to target (not pose angle!)
        thetaTarget = math.atan2((yTarget - X[1,0]),(xTarget - X[0,0]))

        ## calculate desired motion speeds
        velAv = Kv * math.sqrt((xTarget-X[0,0])**2 + (yTarget-X[1,0])**2) 
        velDiff = Kh * (thetaTarget - X[2,0])
        vL = velAv - velDiff/2
        vR = velAv + velDiff/2
        if vL > 0.5:
            vL = 0.5
        if vR > 0.5:
            vR = 0.5
        if vL < -0.5:
            vL = -0.5
        if vR< -0.5:
            vR = -0.5

        ## do movement
        tL1 = mA.get_ticks(); tR1 = mB.get_ticks()
        mA.set_power(speed2powerLeft(vR))
        mB.set_power(speed2powerRight(vL))
        time.sleep(0.2)
        mA.set_power(0)
        mB.set_power(0)
        tL2 = mA.get_ticks(); tR2 = mB.get_ticks()
        
        time.sleep(1)
               
        # print stuff
        print('---------')
        print('vL = '+str(vL))
        print('vR = '+str(vR))
        print('x = ' + str(X[0,0]))
        print('y = ' + str(X[1,0]))
        print('theta = ' + str(X[2,0]))
        print('delta_d = ' + str(delta_d))
        print('delta_th = ' + str(delta_th))
    print('done')   
    mA.set_power(0)
    mB.set_power(0)
    
    plt.show()
    input()
    
    return X[0,0], X[1,0], X[2,0]

if __name__ == '__main__':
    x = 1
    y = 1
    #xCurrent, yCurrent, thetaCurrent = toPoint(x,-y, xCurrent, yCurrent, thetaCurrent)
    #img = capture_image1()
    #key,x,y = getGoalPosPixel(img)
    print('---------')
    print('x = '+str(x))
    print('y = '+str(y))
    xCurrent, yCurrent, thetaCurrent = toPoint(x,y, xCurrent, yCurrent, thetaCurrent)

