import cv2
import numpy as np

if __name__ == '__main__':

    img = cv2.imread('beaconImage229.jpg')
    
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_fg = np.array([45,112,25])         #90 45 10
    upper_fg = np.array([63,180,112])         #126 70 45
    m_fg = cv2.inRange(hsv,lower_fg,upper_fg)
    kernel_saltpepper = np.ones((5,5),np.uint8)
    filtered_m_fg = cv2.morphologyEx(m_fg, cv2.MORPH_CLOSE, kernel_saltpepper)
    cv2.imshow('filtered floor green',filtered_m_fg)
    filtered_m_fg = cv2.bitwise_not(filtered_m_fg)
    #img = cv2.bitwise_or(img, img, mask=filtered_m_fg)
   
    b,g,r = cv2.split(img)
    b = cv2.bilateralFilter(b, 11, 17, 17)#(gray, 11, 17, 17)
    g = cv2.bilateralFilter(g, 11, 17, 17)#(gray, 11, 17, 17)
    r = cv2.bilateralFilter(r, 11, 17, 17)#(gray, 11, 17, 17)
    cv2.imshow('b',b)
    cv2.imshow('g',g)
    cv2.imshow('r',r)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.bilateralFilter(gray, 11, 17, 17)#(gray, 11, 17, 17)
    cv2.imshow('gray',gray)
    edged = cv2.Canny(gray, 30, 200)
    cv2.imshow('edged',edged)
    #cv2.waitKey()
    
    #cnts, hierarchy = cv2.findContours(edged.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    im2, cnts, hierarchy = cv2.findContours(edged.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:10]
    screenCnt = []

    # loop over our contours
    for c in cnts:

        # approximate the contour
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.1 * peri, True)

        print(len(approx))
        # if our approximated contour has four points, then
        # we can assume that we have found our screen
        if len(approx) == 4:
            #print(c)
            screenCnt.append(approx)
            #break
        
    print(screenCnt)
    
    for it in screenCnt:
        cv2.drawContours(img, screenCnt, -1, (0, 255, 0), 3)
    cv2.imshow('contour', img)
    cv2.waitKey(0)