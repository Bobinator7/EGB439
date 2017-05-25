import cv2
import numpy as np

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

if __name__ == '__main__':

    img = cv2.imread('beaconImage337.jpg')
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
        result.append(beaconID)
        
    print(result)

    #TODO: derive blue from other values (scan horizontal line in blue mask until true is found)
    #TODO: calculate beacon range by distance of grouped blobs
    #TODO: calculate beacon angle by offset from centre line

    cv2.waitKey(0);
