import cv2
import numpy as np

cam = cv2.VideoCapture(0)

greenLower = (16, 204, 133) # H
greenUpper = (24, 255, 255)

while True:
    x,frame = cam.read()
    y = cv2.flip(frame,1)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    mask = cv2.inRange(hsv, greenLower, greenUpper)


    mask1 = cv2.erode(mask, None, iterations=20)
    mask1 = cv2.dilate(mask1, None, iterations=20)

    cv2.circle(frame,(100,100),50,(0,255,0),50)
    
    
    # show the frame to our screen
    cv2.imshow("Frame", frame)
    cv2.imshow("HSV",hsv)
    cv2.imshow("MAsk",mask)
    cv2.imshow("Erode/Dilate",mask1)
##
##    print "frame type", type(frame)
##    print "hsv type", type(hsv)
##    print "mask type", type(mask)
##    print "mask1 type", type(mask1)
    key = cv2.waitKey(1) & 0xFF

    # if the 'q' key is pressed, stop the loop
    if key == ord("a"):
            break

cv2.destroyAllWindows()
cam.release()

print("frame type", (frame.shape))
print("hsv type", (hsv.shape))
print("mask type", (mask.shape))
print("mask1 type", (mask1.shape))
