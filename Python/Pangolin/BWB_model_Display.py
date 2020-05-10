# https://github.com/stevenlovegrove/Pangolin/tree/master/examples/HelloPangolin

import OpenGL.GL as gl
import pangolin
import time
import math
import numpy as np
import BWB_model



def main():

    imageW = 640
    imageH = 480
    pangolin.CreateWindowAndBind('Main', imageW, imageH)
    gl.glEnable(gl.GL_DEPTH_TEST)

    # Define Projection and initial ModelView matrix
    scam = pangolin.OpenGlRenderState(
        pangolin.ProjectionMatrix(640, 480, 320, 240, 320, 240, 0.1, 20000),
        pangolin.ModelViewLookAt(0, 500, 0, 0, 0, 0, pangolin.AxisDirection.AxisZ)) #Axis up
    handler = pangolin.Handler3D(scam)

    # Create Interactive View in window
    dcam = pangolin.CreateDisplay()
    dcam.SetBounds(0.0, 1.0, 180/640., 1.0, -640.0/480.0)
    dcam.SetHandler(handler)

    panel = pangolin.CreatePanel('ui')
    panel.SetBounds(0.0, 1.0, 0.0, 180/1000.)

    yaw_slider = pangolin.VarInt('ui.Yaw', value=0, min=-180, max=180)
    button = pangolin.VarBool('ui.Button', value=False, toggle=False)
    Enc_slider = pangolin.VarInt('ui.Encoder', value=0, min=-100, max=100)
    #EncY_slider = pangolin.VarInt('ui.EncY', value=0, min=-1000, max=1000)
    Sho_slider = pangolin.VarInt('ui.Shoulder', value=0, min=-15, max=105)
    El_slider = pangolin.VarInt('ui.Elbow', value=0, min=-160, max=60)

    Xr = 0
    Yr = 0
   
    while not pangolin.ShouldQuit():
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        gl.glClearColor(1.0, 1.0, 1.0, 1.0)
        dcam.Activate(scam)

        if pangolin.Pushed(button):    
            Xr = Xr + float((math.cos(math.radians(yaw_slider.Get())) * Enc_slider.Get()))
            Yr = Yr + float((math.sin(math.radians(yaw_slider.Get())) * Enc_slider.Get()))

        RobotPos = np.array([Xr,Yr,0,1])
        points, CamPose = BWB_model.DHGetRobotPoints(RobotPos, yaw_slider.Get() , Sho_slider.Get(), El_slider.Get())

       
        #Draw Point Cloud
        gl.glPointSize(10)
        gl.glColor3f(1.0, 0.0, 0.0)
        pangolin.DrawPoints(points)

               
        #Draw lines
        gl.glLineWidth(3)
        gl.glColor3f(0.0, 0.0, 0.0)
        pangolin.DrawLine(points)

      
        gl.glLineWidth(1)
        gl.glColor3f(0.0, 0.0, 1.0)
        pangolin.DrawCamera(CamPose, 4, 5, 5)
        

        pangolin.FinishFrame()


if __name__ == '__main__':
    main()
