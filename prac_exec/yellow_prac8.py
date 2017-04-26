## libraries

# std python
import time
import math 
import io

# penguinPi
import penguinPi as ppi
import picamera
import picamera.array

# opencv
import cv2
import numpy as np

##### image processing stuff #####
IMAGE_WIDTH = 320   
IMAGE_HEIGHT=240

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
    
    obj_pix = keypoints[0].size)
    focal = 3.04*1000
    obj_real = 0.1
    
    const
    dist_x = obj_real*focal/obj_pix 
    value = -(keypoints[0].pt[1]-IMAGE_WIDTH/2)*(dist_x * const)
    
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
params_beacon.minThreshold = 10
params_beacon.maxThreshold = 200
params_beacon.filterByColor = True
params_beacon.blobColor = 255
params_beacon.filterByArea = True
params_beacon.minArea = 20
params_beacon.maxArea = 60
params_beacon.filterByCircularity = True
params_beacon.minCircularity = 0.1
params_beacon.filterByInertia = False
params_beacon.filterByConvexity = False
detector_beacon = cv2.SimpleBlobDetector_create(params_beacon)

def getBeaconBasePixels(img,number_of_beacons = 3):
    
    #img = cv2.imread(image_path,1)
    #img = np.asarray(data, dtype=np.uint8)
    gamma_conv = gamma_correction(img,2.2)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    convHSV = cv2.cvtColor(gamma_conv, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(gray,100,255)
    cv2.imshow('mask', mask)
    sobelx = cv2.Sobel(mask,cv2.CV_64F,0,1,ksize=3)
    sobel_gray = cv2.bitwise_and(img,img,mask=np.uint8(np.absolute(sobelx)))
    
    base_points = detector_beacon.detect(sobel_gray)
    
    # return coordinates of beacon bases
    base_points = base_points[:number_of_beacons]
    im_with_keypoints = cv2.drawKeypoints(sobel_gray, base_points, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imshow('keypoints', im_with_keypoints)
    for it in base_points:
        print(it.pt)
    cv2.waitKey(0)
    if base_points == []:
        print('no beacon!')
        return

    slice_width = 4
    pix_coords = np.empty([3,len(base_points)])
    for it in range(0,len(base_points)):
        # get pixel coordinates
        pix_coords[0,it] = int(round(base_points[it].pt[0]))
        pix_coords[1,it] = int(round(base_points[it].pt[1]))
        
        # slice beacon picture
        x_bound = int(pix_coords[0,it]-slice_width/2)
        y_bound = int(pix_coords[1,it])
        slice = convHSV[:y_bound,x_bound:x_bound+slice_width,:]
        
        # get colormasks
        mask_g = cv2.inRange(slice,lower_green,upper_green)
        mask_r1 = cv2.inRange(slice,lower_red1,upper_red1)
        mask_r2 = cv2.inRange(slice,lower_red2,upper_red2)
        mask_r = mask_r1+mask_r2
        mask_b = cv2.inRange(slice,lower_blue,upper_blue)
        
        # start iterating at base and centre of beacon        
        count = -1
        centre = int(mask_g.shape[1]/2)
        flag = [0,0,0]
        col_vec = [0,0,0]
        col_count = 0
        
        # iterate until three colors are found (either blue, green or red)
        # colors are saved if the color is different from the previous color
        while abs(count) < mask_g.shape[0]:
            if col_count >= 3:
                break
            
            # blue case
            if mask_b[count,centre] == 255:
                if flag == [1,0,0]:
                    count = count - 1
                    continue
                else:
                    #print('blue!')
                    flag = [1,0,0]
                    col_vec[col_count] = 3
                    col_count = col_count + 1
                    count = count - 1
                    continue
                    
            # green case
            if mask_g[count,centre] == 255: 
                if flag == [0,1,0]:
                    count = count - 1
                    continue
                else:
                    #print('green!')
                    flag = [0,1,0]
                    col_vec[col_count] = 2
                    col_count = col_count + 1
                    count = count - 1
                    continue
                    
            # red case
            if mask_r[count,centre] == 255: 
                if flag == [0,0,1]:
                    count = count - 1
                    continue
                else:
                    #print('red!')
                    flag = [0,0,1]
                    col_vec[col_count] = 1
                    col_count = col_count + 1
                    count = count - 1
                    continue
                    
            # otherwise inspect next pixel and do nothing
            count = count - 1
            continue
           
        # calculate beacon ID from color code
        if col_count < 3:
            pix_coords[2,it] = 0
        else:
            pix_coords[2,it] = int(col_vec[0] + col_vec[1] * 4 + col_vec[2] * 16)
            #print(col_vec[0] + col_vec[1] * 4 + col_vec[2] * 16)
            
    list = []
    for n in range(0,len(base_points)):
        if pix_coords[2,n] == 0:
            continue
        else:
            list.append(pix_coords[:,n])
    

    # return matrix of base coords of beacon and their respective ID
    return list

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

def moveToBeacon(data,beacon_id):
    
    #data = capture_image()

#    goal_y = getGoalPosPixel(data)[0]
#    print(goal_y)
#    if goal_y == []:
#        print('goal not found!')
    centre = 160 
    goal_tolerance_y = 20
#    while abs(centre-goal_y) > goal_tolerance_y:
        #data = capture_image()
    beacon_list = getBeaconBasePixels(data)
    #print(beacon_list)

    if beacon_list == []:
        print('goal not found!')
        return
    else:
        for it in beacon_list:
            print(it)

#    if (centre > goal_y+goal_tolerance_y):
#        rotate(5, motor_L, motor_R)
#    elif (centre < goal_y-goal_tolerance_y):
#        rotate(5, -motor_L, -motor_R)
#    else:
#        translate(0.1,motor_L,motor_R)



for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    image = np.uint8(frame.array)
    #print(getGoalPosPixel(image)) 
    #moveToBeacon(image,29)
    #data = capture_image1()
    moveToGoal(image)
    #avoidObstacle(image)
    #cv2.waitKey(0)
    rawCapture.truncate(0) 
