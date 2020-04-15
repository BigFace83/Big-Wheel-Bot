import cv2
import numpy as np
import os
import math
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt



#####################################################################################################################################
# Sonar Model
# occmap = map array, Xr = Robot X coordinate, Yr = Robot Y coordinate, Rangle = Robot heading in degrees, SonarDist = Sonar reading in mm
# thickness = Object thickness in mm, Scale = float map scale
#
# Returns: Sonar_log - Array of log odds probabilities. This can be added to existing log odds occ map to update map with latest sonar data.
#          occ - Array of occupied points for scan matching
#####################################################################################################################################
def SonarModel(occmap, Xr, Yr, Rangle, SonarDist, thickness, scale):
   
    Sonar_mask = np.zeros((occmap.shape),np.uint8)
    Sonar_log = np.zeros((occmap.shape),np.single)
    robotpt = (Xr,Yr)

    thickness = int(thickness * scale)
    SonarDist = int(SonarDist * scale)

    cv2.ellipse(Sonar_mask,(int(Xr),int(Yr)),(SonarDist, SonarDist), Rangle, -15, 15, 255, -1) #Draw ellipse on to sonar mask
    pixelpoints = cv2.findNonZero(Sonar_mask) #Find all pixel points in sonar cone

    occ = np.empty((0,1,2),int)

    for x in pixelpoints:    

        Ximg = x[0][0]
        Yimg = x[0][1]
        dist = np.linalg.norm(robotpt - x) #Find euclidean distance to all points in sonar cone from robot location

        theta = (math.degrees(math.atan2((Yimg-Yr),(Ximg-Xr)))) - Rangle #Find angle from robot location to each cell in pixelpoints
        if theta < -180:                                                 #Note:numpy and OpenCV X and Y reversed
            theta = theta + 360
        elif theta > 180:
            theta = theta - 360
    
        sigma_t = 5
        A = 1 / (math.sqrt(2*math.pi*sigma_t))
        C = math.pow((theta/sigma_t),2)
        B = math.exp(-0.5*C)
        Ptheta = A*B
    
        Pdist = (SonarDist - dist/2)/SonarDist
        P = (Pdist*2)*Ptheta


        if dist > SonarDist - thickness and dist < SonarDist + thickness: #occupied region
            Px = 0.5 + Ptheta
            logPx = math.log(Px/(1-Px))
            Sonar_log[Yimg][Ximg] = logPx
            #occ = np.append(occ,[x],0)
        else: #free region
            Px = 0.5 - P
            logPx = math.log(Px/(1-Px))
            Sonar_log[Yimg][Ximg] = logPx
       

    return Sonar_log, occ

#####################################################################################################################################
# IR Model
# occmap = map array, Xr = Robot X coordinate, Yr = Robot Y coordinate, Rangle = Sensor heading in degrees, IRDist = IR reading in mm
# thickness = Object thickness in mm, Scale = float map scale
#
# Returns: IR_log - Array of log odds probabilities. This can be added to existing log odds occ map to update map with latest sonar data.
#          occ - Array of occupied points for scan matching
#####################################################################################################################################
def IRModel(occmap, Xr, Yr, Rangle, IRDist, thickness, scale):
  

    IR_mask = np.zeros((occmap.shape),np.uint8)
    IR_log = np.zeros((occmap.shape),np.single)

    if IRDist == 0:
        print 'IR Zero'
        return IR_log

    robotpt = (Xr,Yr)
    IR_Max = int(600 * scale)

    thickness = int(thickness * scale)
    IRDist = int(IRDist * scale)

    XIR = Xr + float((math.cos(math.radians(Rangle)) * IRDist))
    YIR = Yr + float((math.sin(math.radians(Rangle)) * IRDist)) 

    cv2.line(IR_mask, (int(Xr),int(Yr)), (int(XIR),int(YIR)), 255, 1)
    pixelpoints = cv2.findNonZero(IR_mask) #Find all pixel points in sonar cone

    occ = np.empty((0,1,2),int)

    for x in pixelpoints:
        Ximg = x[0][0]
        Yimg = x[0][1]
        dist = np.linalg.norm(robotpt - x) #Find euclidean distance to all points in sonar cone from robot location
        
        Pdist = (IRDist - (dist/2))/IRDist
        
        if dist > IRDist - thickness and dist < IRDist + thickness: #occupied region
            if IRDist < IR_Max: #Only update occupied region if IR reading is less than maximum.
                Px = 0.8
                logPx = math.log(Px/(1-Px))
                IR_log[Yimg][Ximg] = logPx
                #occ = np.append(occ,[x],0)
        else: #free region
            Px = 0.3#0.5 - (Pdist/3)
            logPx = math.log(Px/(1-Px))
            IR_log[Yimg][Ximg] = logPx
        
    return IR_log, occ



if __name__ == "__main__":

    Xr = float(50)
    Yr = float(50)

    scale = float(1)/100

    cv2.namedWindow("Sonar", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Sonar", 1000, 1000) 
    cv2.waitKey(1)

    Map_log = np.full((100,100),0,np.single)

    #IR_log, occ = IRModel(Map_log, Xr, Yr, -45, 590, 100, scale)
    #Map_log = np.add(Map_log, IR_log)

    Sonar_log, occ = SonarModel(Map_log, Xr, Yr, -90, 4500, 100, scale)
    Map_log = np.add(Map_log, Sonar_log)

    Map_P = 1 - (1/(1+(np.exp(Map_log))))
    Disp_img = 1-Map_P
    cv2.imshow("Sonar", Disp_img)
    cv2.waitKey(1)

    #'''
    #Plot as matplotlib 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')



    #plt.axis('off')
    x = np.arange(0,100,1)
    y = np.arange(0,100,1)
    X, Y = np.meshgrid(x, y)
    ax.plot_wireframe(X, Y, Map_P)
    plt.show()
    #'''

    cv2.waitKey(0) #Leave window open


'''
Sonar_log = SonarModel(Map_log, Xr, Yr, -90, 6000, 100, scale)
Map_log = np.add(Map_log, Sonar_log)
Map_P = 1 - (1/(1+(np.exp(Map_log))))
Disp_img = 1-Map_P

cv2.imshow("Sonar", Disp_img)
cv2.waitKey(1)
'''




    

