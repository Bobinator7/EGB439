import cv2
import numpy as np

def getChrom(img):
    chrom = np.zeros((img.shape[0],img.shape[1],3), np.float32)
    
    for i in range(0,img.shape[0]):
        for j in range(0,img.shape[1]):
            b = img[i,j,0]
            g = img[i,j,1]
            r = img[i,j,2]
            sum = b + g + r  
            chrom[i,j,0] = img[i,j,0] / sum
            chrom[i,j,1] = img[i,j,1] / sum
            chrom[i,j,2] = img[i,j,2] / sum
            
    return chrom

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
                    
            if (maximum-minimum < 20):
                #print(maximum)
                #print(minimum)
                #print(img[i,j,:])
                mask[i,j] = 255
                
    return mask

if __name__ == '__main__':

    img = cv2.imread('beaconImage73.jpg')
    cv2.imshow('orig',img)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    chrom = getChrom(img)
    
    m_gray = getGrayMask(img)
    cv2.imshow('test',m_gray)
    
    ######
    ## color mask limits based on chromaticity values 
#    lower_r = np.array([0,0.08,0.57])
#    upper_r = np.array([1,0.29,0.9])
#    lower_g = np.array([0,0.35,0.35])
#    upper_g = np.array([1,0.51,0.63])
#    lower_b = np.array([0,0.2,0.25])
#    upper_b = np.array([1,0.36,0.43])
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
    lower_r1 = np.array([0,153,50])          #0 60 20
    upper_r1 = np.array([11,255,255])       #22 100 100
    lower_r2 = np.array([178,153,50])        #356 60 20
    upper_r2 = np.array([180,255,255])      #360 100 100
    lower_g = np.array([17,100,50])          #34 40 20
    upper_g = np.array([42,255,130])         #84 100 50
    lower_b = np.array([70,12,25])           #140 5 10
    upper_b = np.array([161,155,130])         #322 60 50
    lower_fg = np.array([45,112,25])         #90 45 10
    upper_fg = np.array([63,180,112])         #126 70 45
    lower_w = np.array([0,0,140])
    upper_w = np.array([180,255,255])

    ## create color masks (note: blue mask inaccurate -> verify blue beacon segment by red and green -> green less precise than red (verify?))
    m_r1 = cv2.inRange(hsv,lower_r1,upper_r1)
    m_r2 = cv2.inRange(hsv,lower_r2,upper_r2)
    m_r = m_r1 + m_r2
    m_g = cv2.inRange(hsv,lower_g,upper_g)
    m_b = cv2.inRange(hsv,lower_b,upper_b)
    m_fg = cv2.inRange(hsv,lower_fg,upper_fg)
    m_w = cv2.inRange(hsv,lower_w,upper_w)
    print(m_w.dtype)
    ###################

    ## remove salt and pepper artifacts
    kernel_saltpepper = np.ones((5,5),np.uint8)
    filtered_m_r = cv2.morphologyEx(m_r, cv2.MORPH_OPEN, kernel_saltpepper)
    filtered_m_r = cv2.morphologyEx(m_r, cv2.MORPH_CLOSE, kernel_saltpepper)
    filtered_m_g = cv2.morphologyEx(m_g, cv2.MORPH_OPEN, kernel_saltpepper)
    filtered_m_g = cv2.morphologyEx(m_g, cv2.MORPH_CLOSE, kernel_saltpepper)
    filtered_m_b = cv2.morphologyEx(m_b, cv2.MORPH_OPEN, kernel_saltpepper)
    filtered_m_b = cv2.morphologyEx(m_b, cv2.MORPH_CLOSE, kernel_saltpepper)
    filtered_m_fg = cv2.morphologyEx(m_fg, cv2.MORPH_OPEN, kernel_saltpepper)
    filtered_m_fg = cv2.morphologyEx(m_fg, cv2.MORPH_CLOSE, kernel_saltpepper)
    
    kernel_dilate = np.ones((5,5),np.uint8) 
    m_background = cv2.dilate(m_w,kernel_dilate,iterations = 1)
    m_background = cv2.bitwise_not(m_background)
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
    bbd.minArea = 20 
    bbd.maxArea = 1000
    bbd.filterByCircularity = True
    bbd.minCircularity = 0.5
    bbd.filterByInertia = False
    bbd.filterByConvexity = False
    detect_beacon = cv2.SimpleBlobDetector_create(bbd)
    '''
    br = detect_beacon.detect(filtered_m_r)
    bg = detect_beacon.detect(filtered_m_g)

    test_r = cv2.drawKeypoints(img,br,np.array([]),(0,0,255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imshow('kp', test_r)
    test_g = cv2.drawKeypoints(img,bg,np.array([]),(0,0,255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imshow('kg', test_g)    
    '''
    #TODO: ignore blobs detected on edge of image (noise outside the arena)
    #TODO: summarize blobs into groups with similiar horizontal value
    #TODO: derive blue from other values (scan horizontal line in blue mask until true is found)
    #TODO: calculate beacon id
    #TODO: use floor mask to calculate beacon position (scan down until floor pixel is hit)

    cv2.waitKey(0);
