import time
import serial
import os
import numpy as np
import cv2
import RPi.GPIO as GPIO


print "Starting..."

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(17,GPIO.OUT) #Red LED
GPIO.setup(22,GPIO.OUT) #Blue LED
GPIO.setup(27,GPIO.OUT) #Green LED

GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_UP) #Power off button

recording = False

print ("Opening Arduino connection")
serArd = serial.Serial('/dev/ttyUSB0', 115200, timeout = 10)
time.sleep(2) # wait here a bit, let arduino boot up fully before sending data
print ("Port open:  " + serArd .portstr)       # check which port was really used
serArd.flushInput()

print ("Opening HC05 connection")
serHC05 = serial.Serial('/dev/ttyAMA0', 9600, timeout = 10)
time.sleep(2) # wait here a bit, let arduino boot up fully before sending data
print ("Port open:  " + serHC05 .portstr)       # check which port was really used
serHC05.flushInput()

print "Starting OpenCV"
capture = cv2.VideoCapture(0)

imagewidth = 320
imageheight = 240
capture.set(3,imagewidth) #1024 640 1280 800 384
capture.set(4,imageheight) #600 480 960 600 288

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')

cv2.waitKey(50)

def CaptureSaveFrame(outfile):
    ret,img = capture.read()
    ret,img = capture.read() #get a bunch of frames to make sure current frame is the most recent
    outfile.write(img)
    cv2.waitKey(1)
    return img

def LEDBlue():
    GPIO.output(17,GPIO.HIGH) 
    GPIO.output(22,GPIO.HIGH) 
    GPIO.output(27,GPIO.LOW)
    
def LEDRed():
    GPIO.output(17,GPIO.HIGH)
    GPIO.output(22,GPIO.LOW)
    GPIO.output(27,GPIO.HIGH)

def LEDGreen():
    GPIO.output(17,GPIO.LOW)
    GPIO.output(22,GPIO.HIGH)
    GPIO.output(27,GPIO.HIGH)

def LEDOff():
    GPIO.output(17,GPIO.HIGH)
    GPIO.output(22,GPIO.HIGH)
    GPIO.output(27,GPIO.HIGH)

def CreateFiles():
    timestr = time.strftime("%Y%m%d-%H%M%S")
    os.chdir("/home/pi/RC_Robot")
    try:
        os.mkdir(timestr, 0o777)
    except OSError:
        print "Creation of directory failed"

    os.chdir(timestr)
    out = cv2.VideoWriter(timestr +'.avi',fourcc, 5.0, (imagewidth,imageheight))
    f = open("Data.txt", 'w')
    return out, f

def Shutdown():
    print("Shutting Down")
    LEDOff()
    time.sleep(0.5)
    LEDBlue()
    time.sleep(0.5)
    LEDOff()
    time.sleep(0.5)
    LEDBlue()
    time.sleep(0.5)
    LEDOff()
    time.sleep(0.5)
    LEDBlue()
    time.sleep(0.5)
    LEDOff()
    time.sleep(0.5)
    LEDRed()
    os.system("sudo shutdown -h now")

def readserial():
    #while True:
    while serHC05.inWaiting()>0: #while data is available
        line = serHC05.readline() # keep reading until...
        if serHC05.inWaiting()==0: #...all data is read. Previous data is discarded to keep robot responsive
            return line.rstrip('\r\n') #Return most recent data
    

def sendcommand(serialdata):
    serArd.write(serialdata + '\n') #Write data string, newline terminated

def readserialArd():
    if serArd.inWaiting()>0: #If data is available
        line = serArd.readline()
        return line.rstrip('\r\n') #Return packet
    else: #If no data to read, return None
        return None

def closeserial():
    ser.close()

def LEDredflash10(): #Flash LED red for 10 seconds. Used as a delay before recording
    for x in range (0, 10, 1):
        LEDRed()
        time.sleep(0.3)
        LEDOff()
        time.sleep(0.7)

GPIO.add_event_detect(21, GPIO.FALLING, callback=Shutdown, bouncetime=2000)
LEDGreen()
SD_count = 0

while True:

    data = readserial()
    
    if data is not None:
        print data
        sendcommand(data)
        dataarray = [int(x) for x in data.split(',')]
        if dataarray[5] == 0:
            recording  = not recording
            time.sleep(0.5) #debounce
            if recording:
                LEDredflash10()
                LEDRed()
                out, f = CreateFiles()
            else:
                LEDGreen()
                f.close()
        print dataarray
        
        if dataarray[4] == 0: #Shutdown Pi if both buttons are pressed
            #if dataarray[5] == 0:
            SD_count += 1
            if SD_count > 20:
                Shutdown()
        else:
            SD_count = 0



    arddata = readserialArd()
    if arddata is not None:
        mylist = [int(x) for x in arddata.split(',')]
        print mylist
        if recording:
            CaptureSaveFrame(out)
            f.write(arddata + '\n')


    




 





