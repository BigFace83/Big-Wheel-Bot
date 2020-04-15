import numpy as np
import cv2

cap = cv2.VideoCapture('/home/peter/Big_Wheel_Bot/Data/20191221-173029/20191221-173029.avi')

while(cap.isOpened()):
    ret, frame = cap.read()

    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    cv2.imshow('frame',frame)
    if cv2.waitKey(200) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
