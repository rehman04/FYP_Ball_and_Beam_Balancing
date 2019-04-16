import cv2
import numpy as np

cam = cv2.VideoCapture(0)

while True:
    x,y = cam.read()
    frame = cv2.flip(y,1)
    print(type(y))
    # show the frame to our screen
    cv2.imshow("Frame", y)
    cv2.imshow("Flipped",frame)
    key = cv2.waitKey(1) & 0xFF

    # if the 'q' key is pressed, stop the loop
    if key == ord("a"):
            break

cv2.destroyAllWindows()
cam.release()
