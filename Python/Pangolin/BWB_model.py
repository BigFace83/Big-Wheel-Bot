import math
import numpy as np




def DHMatrix(a, alpha, d, theta):
    cos_theta = math.cos(math.radians(theta))
    sin_theta = math.sin(math.radians(theta))
    cos_alpha = math.cos(math.radians(alpha))
    sin_alpha = math.sin(math.radians(alpha))


    return np.array([
        [cos_theta, -sin_theta*cos_alpha, sin_theta*sin_alpha, a*cos_theta],
        [sin_theta, cos_theta*cos_alpha, -cos_theta*sin_alpha, a*sin_theta],
        [0, sin_alpha, cos_alpha, d],
        [0, 0, 0, 1],
    ])

def DHGetRobotPoints(RobotPos, thetaBase, thetaLower, thetaElbow):

    T0 = np.array([0,0,0,1])
    T1 = DHMatrix(95, 90, 0, thetaBase) #Shoulder joint
    T2 = DHMatrix(150, 0, 0, thetaLower) #Elbow joint
    T3 = DHMatrix(155, 0, 0, thetaElbow) #Arm end
    T4 = DHMatrix(60, 0, 0, thetaElbow) #Camera mount position on arm
    T5 = DHMatrix(40, 90, 0, 90) #Camera

    XYZ1 = np.dot(T1,T0)
    XYZ2 = np.dot(T1,np.dot(T2,T0))
    XYZ3 = np.dot(T1,np.dot(T2,np.dot(T3,T0)))
    XYZ4 = np.dot(T1,np.dot(T2,np.dot(T4,T0)))
    XYZ5 = np.dot(T1,np.dot(T2,np.dot(T4,np.dot(T5,T0))))

  
    EncXarr = np.array([[1,0,0,RobotPos[0]], #Translation matrix to robot position
                        [0,1,0,RobotPos[1]],
                        [0,0,1,RobotPos[2]+200], #Shoulder joint is 200mm higher in the z direction than the ground
                        [0,0,0,1]])

    XYZ1 = np.dot(EncXarr,XYZ1) #Move all calculated points relative to robot XYZ position
    XYZ2 = np.dot(EncXarr,XYZ2)
    XYZ3 = np.dot(EncXarr,XYZ3)
    XYZ4 = np.dot(EncXarr,XYZ4)
    XYZ5 = np.dot(EncXarr,XYZ5)

    CamPose = np.dot(T1,np.dot(T2,np.dot(T4,T5))) #Use pose from XYZ5 but do not translate
    CamPose[0, 3] = XYZ5[0] #Set XYZ for camera pose matrix manually
    CamPose[1, 3] = XYZ5[1]
    CamPose[2, 3] = XYZ5[2]

    #Form return array as a 2D array of robot points
    ReturnArray = np.array([RobotPos, #Robot coordinates ground level central point between wheels
                            #np.array([RobotPos[0],RobotPos[1],RobotPos[2]+200,1]),
                            XYZ1,  #Shoulder coordinates
                            XYZ2,  #Elbow coordinates
                            XYZ3,  #Arm end
                            XYZ4,  #Camera mount coordinates
                            XYZ5]) #Camera

    return ReturnArray, CamPose
