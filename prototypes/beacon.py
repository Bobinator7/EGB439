import cv2
import numpy as np

if __name__ == '__main__':

    img = cv2.imread('beaconImage292.jpg')
    cv2.imshow('orig',img)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    ## color mask limits based on hsv values (gimp format: 0-360 0-100 0-100)
    lower_r1 = np.array([0,153,50])          #0 60 20
    upper_r1 = np.array([11,255,255])       #22 100 100
    lower_r2 = np.array([178,153,50])        #356 60 20
    upper_r2 = np.array([180,255,255])      #360 100 100
    lower_g = np.array([17,100,50])          #34 40 20
    upper_g = np.array([42,255,130])         #84 100 50
    lower_b = np.array([70,12,25])           #140 5 10
    upper_b = np.array([161,155,130])         #322 60 50
    #lower_fg = np.array([45,112,25])         #90 45 10
    #upper_fg = np.array([63,180,112])         #126 70 45

    ## create color masks (note: blue mask inaccurate -> verify blue beacon segment by red and green -> green less precise than red (verify?))
    m_r1 = cv2.inRange(hsv,lower_r1,upper_r1)
    m_r2 = cv2.inRange(hsv,lower_r2,upper_r2)
    m_r = m_r1 + m_r2
    m_g = cv2.inRange(hsv,lower_g,upper_g)
    m_b = cv2.inRange(hsv,lower_b,upper_b)
    #m_fg = cv2.inRange(hsv,lower_fg,upper_fg)

    ## remove salt and pepper artifacts
    kernel_saltpepper = np.ones((5,5),np.uint8)
    filtered_m_r = cv2.morphologyEx(m_r, cv2.MORPH_OPEN, kernel_saltpepper)
    filtered_m_r = cv2.morphologyEx(m_r, cv2.MORPH_CLOSE, kernel_saltpepper)
    filtered_m_g = cv2.morphologyEx(m_g, cv2.MORPH_OPEN, kernel_saltpepper)
    filtered_m_g = cv2.morphologyEx(m_g, cv2.MORPH_CLOSE, kernel_saltpepper)
    filtered_m_b = cv2.morphologyEx(m_b, cv2.MORPH_OPEN, kernel_saltpepper)
    filtered_m_b = cv2.morphologyEx(m_b, cv2.MORPH_CLOSE, kernel_saltpepper)
    #filtered_m_fg = cv2.morphologyEx(m_fg, cv2.MORPH_OPEN, kernel_saltpepper)
    #filtered_m_fg = cv2.morphologyEx(m_fg, cv2.MORPH_CLOSE, kernel_saltpepper)
    
    ## debug filtered image
    #cv2.imshow('red',m_r)
    cv2.imshow('filtered red',filtered_m_r)
    #cv2.imshow('green',m_g)
    cv2.imshow('filtered green',filtered_m_g)
    #cv2.imshow('blue',m_b)
    cv2.imshow('filtered blue',filtered_m_b)
    #cv2.imshow('floor',m_fg)
    #cv2.imshow('filtered floor green',filtered_m_fg)

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

    br = detect_beacon.detect(filtered_m_r)
    bg = detect_beacon.detect(filtered_m_g)

    test_r = cv2.drawKeypoints(img,br,np.array([]),(0,0,255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imshow('kp', test_r)
    test_g = cv2.drawKeypoints(img,bg,np.array([]),(0,0,255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imshow('kg', test_g)

    #TODO: ignore blobs detected on edge of image (noise outside the arena)
    #TODO: summarize blobs into groups with similiar horizontal value
    #TODO: derive blue from other values (scan horizontal line in blue mask until true is found)
    #TODO: calculate beacon id
    #TODO: use floor mask to calculate beacon position (scan down until floor pixel is hit)

    cv2.waitKey(0);
