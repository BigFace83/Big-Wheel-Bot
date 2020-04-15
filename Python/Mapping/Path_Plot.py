import cv2
import numpy as np
import os
import math


os.chdir('Data/Garden_09_04_20') #Path to data file

Xr = float(250) #Set starting position for robot
Yr = float(250)

prevangle = 0
Sonarangle = 15

scale = float(1)/100

cv2.namedWindow("Path Map", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Path Map", 1000, 1000) 
cv2.waitKey(50)

map_image = np.zeros((500,500,3),np.uint8)

cv2.waitKey(50)

f = open("Data.txt", "r")
for line in f:
    mylist = [int(x) for x in line.split(',')]
  
    angle = mylist[2]
    avencoder = (mylist[0]+mylist[1])/2
    Fsonar = mylist[3]*10
    Rsonar = mylist[4]*10
    LIR = mylist[5]*10
    RIR = mylist[6]*10

    deltaangle = (angle - prevangle)/2
    
    Xrnext = Xr + float((math.cos(math.radians(angle + deltaangle)) * (avencoder*5)*scale))
    Yrnext = Yr + float((math.sin(math.radians(angle + deltaangle)) * (avencoder*5)*scale))

    print Xrnext

    '''
    scaledFs = int (Fsonar*scale)
    scaledRs = int (Rsonar*scale)

    #cv2.ellipse(map_image,(int(Xr),int(Yr)),(scaledFs,scaledFs), angle, -Sonarangle, Sonarangle, (0,255,0), -1)
    Xsf = Xr + float((math.cos(math.radians(angle)) * Fsonar*scale))
    Ysf = Yr + float((math.sin(math.radians(angle)) * Fsonar*scale))

    #cv2.ellipse(map_image,(int(Xr),int(Yr)),(scaledRs,scaledRs), angle, -Sonarangle, Sonarangle, (0,255,0), -1)
    Xsr = Xr + float((math.cos(math.radians(angle)) * Rsonar*scale))
    Ysr = Yr + float((math.sin(math.radians(angle)) * Rsonar*scale))

    XLIR = Xr + float((math.cos(math.radians(angle-45)) * LIR*scale))
    YLIR = Yr + float((math.sin(math.radians(angle-45)) * LIR*scale)) 

    XRIR = Xr + float((math.cos(math.radians(angle+45)) * RIR*scale))
    YRIR = Yr + float((math.sin(math.radians(angle+45)) * RIR*scale)) 

    cv2.line(map_image, (int(Xr),int(Yr)), (int(Xsf),int(Ysf)), (0,255,0), 1)
    cv2.line(map_image, (int(Xr),int(Yr)), (int(Xsr),int(Ysr)), (0,255,0), 1)
    cv2.line(map_image, (int(Xr),int(Yr)), (int(XLIR),int(YLIR)), (0,255,0), 1)
    cv2.line(map_image, (int(Xr),int(Yr)), (int(XRIR),int(YRIR)), (0,255,0), 1)
    '''

    cv2.line(map_image, (int(Xr),int(Yr)), (int(Xrnext),int(Yrnext)), (0,0,255), 1)

    Xr = Xrnext
    Yr = Yrnext

    prevangle = angle

    cv2.imshow("Path Map", map_image)
    cv2.waitKey(10)

cv2.waitKey(0)

