#WORKING
## libraries

# std python
import time
import math 
import io

# penguinPi
import penguinPi as ppi
import picamera
import picamera.array
#import imgProc_vFinal.py as imgProc
# opencv
import cv2
import numpy as np

##### image processing stuff #####

## general
def gamma_correction(img, correction):
    img = img/255.0
    img = cv2.pow(img, correction)
    return np.uint8(img*255)

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
    
    # return yellow blob, if some detected
    if keypoints != []:
        #im_with_keypoints = cv2.drawKeypoints(masked[:,:,2], keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        #cv2.imshow('keypoints', im_with_keypoints)
        #cv2.waitKey(0)
        #print(keypoints[0].pt) 
        return np.floor(keypoints[0].pt).astype(int)

    # return empty list otherwise
    return []

## beacon stuff
lower_green = np.array([30,70,10])
upper_green = np.array([90,255,255])
lower_red1 = np.array([0,110,25])
upper_red1 = np.array([30,255,255])
lower_red2 = np.array([150,110,25])
upper_red2 = np.array([180,255,255])
lower_blue = np.array([90,70,10])
upper_blue = np.array([150,255,255])

params_beacon = cv2.SimpleBlobDetector_Params()
params_beacon.minThreshold = 50 #20
params_beacon.maxThreshold = 200
params_beacon.filterByColor = True
params_beacon.blobColor = 255
params_beacon.filterByArea = True
params_beacon.minArea = 30 #20
params_beacon.maxArea = 60
params_beacon.filterByCircularity = True
params_beacon.minCircularity = 0.2 #0.1
params_beacon.filterByInertia = False
params_beacon.filterByConvexity = False
detector_beacon = cv2.SimpleBlobDetector_create(params_beacon)

def chunks(l, n):
    """Yield successive n-sized chunks from l."""
    tolerance = 5
    
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

def getBeaconIDs(img):
    
    # get image and correct color space
    imgb,imgg,imgr = cv2.split(img)
    
    gamma_conv = gamma_correction(img,2.2)
    
    #for x in range(0,240):
    #    for y in range(0,320):
    #        if imgb[x,y] == 0:
    #            imgb[x,y] = 1
    #        if imgr[x,y] == 0:
    #            imgr[x,y] = 1
    #        if imgg[x,y] == 0:
    #             imgg[x,y] = 1
 
    imgCromb = np.uint8(imgb/(imgb+imgg+imgr)*255)
    imgCromg = np.uint8(imgg/(imgb+imgg+imgr)*255)
    imgCromr = np.uint8(imgr/(imgb+imgg+imgr)*255)
    
    #cv2.imshow('Crom b', imgCromb)
    #cv2.imshow('Crom r', imgCromr)
    #cv2.imshow('Crom g', imgCromg)
    
    #cv2.imwrite('imgCromb.jpg', imgCromb)
    #cv2.imwrite('imgCromr.jpg', imgCromr)
    #cv2.imwrite('imgCromg.jpg', imgCromg)
       
    #imgCromb = cv2.convertScaleAbs(imgCromb, 255)
    #imgCromg = cv2.convertScaleAbs(imgCromg, 255)
    #imgCromr = cv2.convertScaleAbs(imgCromr, 255)
    
    gray = cv2.cvtColor(gamma_conv, cv2.COLOR_BGR2GRAY)
    
    ret,thb = cv2.threshold(imgCromb,55,255,cv2.THRESH_BINARY) #55
    ret,thg = cv2.threshold(imgCromg,70,255,cv2.THRESH_BINARY) #70
    ret,thr = cv2.threshold(imgCromr,110,255,cv2.THRESH_BINARY)
    
    ret,th_gray = cv2.threshold(gray,60,255,cv2.THRESH_BINARY)
    
    kernel = np.ones((5,5),np.uint8)
    th_gray = cv2.morphologyEx(th_gray, cv2.MORPH_OPEN, kernel)
    th_gray = cv2.bitwise_not(th_gray)
    

    kernelC = np.ones((1,40), np.uint8) #30
    thb2 = cv2.morphologyEx(thb, cv2.MORPH_CLOSE, kernelC)
    thg2 = cv2.morphologyEx(thg, cv2.MORPH_CLOSE, kernelC)
    thr2 = cv2.morphologyEx(thr, cv2.MORPH_CLOSE, kernelC)
    thb2 = cv2.morphologyEx(thb2, cv2.MORPH_OPEN, kernel)
    thg2 = cv2.morphologyEx(thg2, cv2.MORPH_OPEN, kernel)
    thr2 = cv2.morphologyEx(thr2, cv2.MORPH_OPEN, kernel)
    
    thb = cv2.bitwise_and(thb,thb,mask=thb2)
    thg = cv2.bitwise_and(thg,thg,mask=thg2)
    thr = cv2.bitwise_and(thr,thr,mask=thr2)
    # convert to HSV space for color detection
    convHSV = cv2.cvtColor(gamma_conv, cv2.COLOR_BGR2HSV)
    
    l = np.array([0,0,0])
    u = np.array([180,255,100])
    thbackground = cv2.inRange(convHSV,l,u)
    #cv2.imshow('back',thbackground)
    
    green = 60
    tolerance_g = 15
    lower_green = np.array([green - tolerance_g,0,0])
    upper_green = np.array([green + tolerance_g,255,255])
    thgreen = cv2.inRange(convHSV,lower_green,upper_green)
    kernel = np.ones((7,7),np.uint8)
    thgreen = cv2.morphologyEx(thgreen, cv2.MORPH_OPEN, kernel)
    thgreen = cv2.bitwise_not(thgreen)
    
    thb = cv2.convertScaleAbs(thb)
    thg = cv2.convertScaleAbs(thg)
    thr = cv2.convertScaleAbs(thr)
    thb = cv2.bitwise_and(thb,thgreen)
    thg = cv2.bitwise_and(thg,thgreen)
    thr = cv2.bitwise_and(thr,thgreen)
    thb = cv2.bitwise_and(thb,th_gray)
    thg = cv2.bitwise_and(thg,th_gray)
    thr = cv2.bitwise_and(thr,th_gray)
    
    #thb = cv2.erode(thb,kernel,iterations = 2)
    #thb = cv2.dilate(thb,kernel,iterations = 2)
    
    thb = cv2.morphologyEx(thb, cv2.MORPH_OPEN, kernel)
    thg = cv2.morphologyEx(thg, cv2.MORPH_OPEN, kernel)
    thr = cv2.morphologyEx(thr, cv2.MORPH_OPEN, kernel)
    
    params = cv2.SimpleBlobDetector_Params()
    params.minThreshold = 10
    params.maxThreshold = 200
    params.filterByColor = True
    params.blobColor = 255
    params.filterByArea = True
    params.minArea = 0
    params.maxArea = 10000
    params.filterByCircularity = True
    params.minCircularity = 0.5
    params.filterByInertia = False
    params.filterByConvexity = False

    frame_tolerance = 10
    detector = cv2.SimpleBlobDetector_create(params)
    thb = cv2.cvtColor(thb, cv2.COLOR_GRAY2BGR)
    keypointsb = detector.detect(thb)
    keypointsb = [i for i in keypointsb if int(round(i.pt[0])) >= frame_tolerance]
    keypointsb = [i for i in keypointsb if int(round(i.pt[1])) >= frame_tolerance]
    keypointsb = [i for i in keypointsb if int(round(i.pt[0])) < thb.shape[1] - frame_tolerance]
    keypointsb = [i for i in keypointsb if int(round(i.pt[1])) < thb.shape[0] - frame_tolerance]
    
    detector = cv2.SimpleBlobDetector_create(params)
    thg = cv2.cvtColor(thg, cv2.COLOR_GRAY2BGR)
    keypointsg = detector.detect(thg)
    keypointsg = [i for i in keypointsg if int(round(i.pt[0])) >= frame_tolerance]
    keypointsg = [i for i in keypointsg if int(round(i.pt[1])) >= frame_tolerance]
    keypointsg = [i for i in keypointsg if int(round(i.pt[0])) < thg.shape[1] - frame_tolerance]
    keypointsg = [i for i in keypointsg if int(round(i.pt[1])) < thg.shape[0] - frame_tolerance]
    
    detector = cv2.SimpleBlobDetector_create(params)
    thr = cv2.cvtColor(thr, cv2.COLOR_GRAY2BGR)
    keypointsr = detector.detect(thr)
    keypointsr = [i for i in keypointsr if int(round(i.pt[0])) >= frame_tolerance]
    keypointsr = [i for i in keypointsr if int(round(i.pt[1])) >= frame_tolerance]
    keypointsr = [i for i in keypointsr if int(round(i.pt[0])) < thr.shape[1] - frame_tolerance]
    keypointsr = [i for i in keypointsr if int(round(i.pt[1])) < thr.shape[0] - frame_tolerance]
    
#    im_with_keypoints = cv2.drawKeypoints(thr, keypointsr, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
#    cv2.imshow('keypoints', im_with_keypoints)
#    for it in keypointsr:
#        print(it.pt)
#    cv2.waitKey(0)

    kp_list = []
    for it in keypointsb:
        kp_list.append([it.pt[0], it.pt[1], 3, it.size])
        
    for it in keypointsg:
        kp_list.append([it.pt[0], it.pt[1], 2, it.size])
        
    for it in keypointsr:
        kp_list.append([it.pt[0], it.pt[1], 1, it.size])
    
    kp_list.sort()
    #for it in kp_list:
    #    print(it)
        
    beacon_list = chunks(kp_list,3)
    result = []
    for it in beacon_list:
        
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
        result.append(beaconID)
        
    

    cv2.imshow('orig',img)
    #cv2.imshow('gamma_conv',gamma_conv)
    #cv2.imshow('b',imgb)
    cv2.imshow('b_thres',thb)
    #cv2.imshow('g',imgg)
    cv2.imshow('g_thres',thg)
    #cv2.imshow('r',imgr)
    cv2.imshow('r_thres',thr)
    #cv2.imshow('gray_th',th_gray)
    #cv2.waitKey(0)
    return result

##### movement stuff #####
ppi.init()
mA = ppi.Motor(ppi.AD_MOTOR_A)
mB = ppi.Motor(ppi.AD_MOTOR_B)
wheel_dia = 0.065
turn_dia = 0.145
p = 45
motor_L = -p
motor_R = -p
error = 4

def enc_diff(init_count, current_count):
    half_res = 15360
    full_res = 30720
    scaled_count = current_count - init_count + half_res
    return_count = current_count - init_count
    if (scaled_count < 0):
        return_count = (half_res - init_count) + (current_count + half_res)# TODO (half_res - init_count) + (current_count + half_res) #scaled_count = scaled_count + full_res
    elif (scaled_count >= full_res):
        return_count = (half_res - current_count) + (init_count + half_res) #scaled_count = scaled_count - full_res
    return return_count

def translate(distance, motor_L, motor_R):
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



##### camera stuff #####

IMAGE_WIDTH = 320   
IMAGE_HEIGHT=240
camera = picamera.PiCamera()
camera.framerate = 3 
camera.resolution = (IMAGE_WIDTH, IMAGE_HEIGHT)
rawCapture = picamera.array.PiRGBArray(camera, size=(IMAGE_WIDTH, IMAGE_HEIGHT))

##### main code #####

stream = io.BytesIO()
def capture_image1():
    camera.capture(stream, format='jpeg', use_video_port=True)
    data = cv2.imdecode(np.fromstring(stream.getvalue(), dtype=np.uint8), 1)   
    return data

def avoidObstacle(image_array):
    #image_array = image_capture1()
    gamma_conv = gamma_correction(image_array,2.2)
    convHSV = cv2.cvtColor(gamma_conv, cv2.COLOR_BGR2HSV)
    ## Gather the red element in redThresh
    lower_red = np.array([0,110,100])
    upper_red = np.array([15,255,255])
    mask_r1 = cv2.inRange(convHSV,lower_red,upper_red)
    lower_red = np.array([150,110,100])
    upper_red = np.array([180,255,255])
    mask_r2 = cv2.inRange(convHSV,lower_red,upper_red)
    mask_r = mask_r1+mask_r2
  
    
    barLocation = 220
    redBar = mask_r[barLocation][:]
    safe_d = 30
    for x in range(0, mask_r.shape[0]):
        if (redBar[x] > 0):
            flag = 1
            if (x >= 180):
                rotate(90,motor_L, motor_R)
                translate(0.1, motor_L, motor_R)
                rotate(90, -motor_L, -motor_R)
                print('Turning Left')
                break
            elif(x<180):
                rotate(90, -motor_L, -motor_R)
                translate(0.1, motor_L, motor_R)
                rotate(90,motor_L, motor_R)
                print('Turning Right')
                break   
        else:
            flag = 0
            #print('No red object detected.')
    return flag

def moveToGoal(data):
    
    #data = capture_image()

#    goal_y = getGoalPosPixel(data)[0]
#    print(goal_y)
#    if goal_y == []:
#        print('goal not found!')
    centre = 160 
    goal_tolerance_y = 20
#    while abs(centre-goal_y) > goal_tolerance_y:
        #data = capture_image()
    goal = getGoalPosPixel(data)
    print(goal)

    if goal == []:
        print('goal not found!')
        return 
    else:
        goal_y = goal[0]
        goal_x = goal[1]
#            break

    bottom = 200
    if (goal_x > bottom):
        return

    if (centre > goal_y+goal_tolerance_y):
        rotate(5, motor_L, motor_R)
    elif (centre < goal_y-goal_tolerance_y):
        rotate(5, -motor_L, -motor_R)
    else:
        flag = avoidObstacle(data)
        if (flag == 0):
            translate(0.1,motor_L,motor_R)

def moveToBeacon(img,beacon_id):
    
    #data = capture_image()

#    goal_y = getGoalPosPixel(data)[0]
#    print(goal_y)
#    if goal_y == []:
#        print('goal not found!')
    centre = 160 
    goal_tolerance_y = 25
    #while abs(centre-goal_y) > goal_tolerance_y:
        #data = capture_image()
    beacon_list = getBeaconIDs(img)
    #print(beacon_list)

    if beacon_list == []:
        print('goal not found!')
        return
    else:
        for it in beacon_list:
            print(it[3])
            if beacon_id == it[2]:
                print('found THE beacon')
                if (centre > it[0]+goal_tolerance_y):
                    rotate(5, motor_L, motor_R)
                elif (centre < it[0]-goal_tolerance_y):
                    rotate(5, -motor_L, -motor_R)
                else:
                    print(it[3])
                    if it[1] < 160:
                        translate(0.1, motor_L, motor_R)
            else:
                print('nothing')
    print(beacon_list)
                #print(beacon_list)
#            else:
                #print('fuck!')
        #for x in range(len(beacon_list)):
            #if (beacon_id == beacon_list[]):
                
        #for it in beacon_list:
            #print(it)
    
#        rotate(5, motor_L, motor_R)
#    elif (centre < goal_y-goal_tolerance_y):
#        rotate(5, -motor_L, -motor_R)
#    else:
#        translate(0.1,motor_L,motor_R)
    return beacon_list

i=0
imageBeaconList =[]
#translate(0.1, motor_L, motor_R)
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
#for i in range(0,300,20):
    image = np.uint8(frame.array)
    #print(getGoalPosPixel(image)) 
    #image=cv2.imread('./newImages/beaconImage'+str(i)+'.jpg')
    #cv2.imwrite('beaconImage'+str(i)+'.jpg',image)
    moveToBeacon(image, 27)
    #imageBeaconList.append(moveToBeacon(cv2.imread('./selectedNewImages/beaconImage'+str(i)+'.jpg'),29))
    #data = capture_image1()
    #moveToGoal(image)
    #avoidObstacle(image)
    #cv2.waitKey(0)
    rawCapture.truncate(0) 
    #cv2.waitKey(1)
    #i=i+1

#print(imageBeaconList)

