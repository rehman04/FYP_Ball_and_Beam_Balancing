# import the necessary packages
from __future__ import division
from collections import deque
import numpy as np
import argparse
import imutils
import cv2
import math
import time
import serial
import struct

#Arduinoo Connection
#arduino = serial.Serial('COM11', 9600)
# let it initialize
time.sleep(2)

#PID Parameters
kp = 10
ki = 0.5
kd = 0
error = 0
lasterror = 0
sum_error = 0
diff_error = 0
anti_windup = 10000
dt = 0
t = 0

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
	help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (16, 204, 133)
greenUpper = (24, 255, 255)
pts = deque(maxlen=args["buffer"])

# if a video path was not supplied, grab the reference
# to the webcam
if not args.get("video", False):
	camera = cv2.VideoCapture(0)

# otherwise, grab a reference to the video file
else:
	camera = cv2.VideoCapture(args["video"])

def draw_angle(center,frame):
    y,x = int(frame.shape[0]/2) , int(frame.shape[1]/2)
#    x_,y_ = frame.shape
    cv2.line(frame,(x,0),(x,y*2),(255,0,0),2)
    cv2.line(frame,(x,0),center,(255,0,0),2)

    #Angle Calculation
    #line 1 (vertical line) consider as x-axis
    #point1: (0,x) | point2: (y*2, x)
    #line 2 (ball line) consider as moving line
    #point1: (0,x) | point2: (center[1],center[0])
    
    hyp = math.sqrt((center[1] - 0)**2 + (center[0] - x)**2)
    perp = center[0]-x

    angle = math.sin(perp/hyp)
    angle = math.degrees(angle)
    #print angle

    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame,str(angle),(x,25), font, 1,(255,255,255),1,cv2.LINE_AA)
    return angle
    
def calculate_pid(angle):        
        global t,kp,ki,kd,error,lasterror,diff_error,sum_error,dt,anti_windup
        setpoint = 0
        dt = time.time() - t
        t = time.time()
        diff_error = error - lasterror
        lasterror = error
        error = setpoint - angle
        correction = kp*error + kd*diff_error/dt + ki*sum_error*dt

        if sum_error < anti_windup:
                sum_error = sum_error + error
        else:
                sum_error = anti_windup

        if correction > 255:
                correction = 255
        if correction < -255:
                correction = -255

        direction = 'n'
        speed = 0
        if correction < 0:
            direction = 'a'
            speed = int(abs(correction))

        if correction > 0:
            direction = 'c'
            speed = int(abs(correction))
            
##        arduino.write(struct.pack('>B',speed))
##        arduino.write(",")
##        arduino.write(direction)
##        arduino.write("#")
        
        #print speed,direction

# keep looping
while True:
	# grab the current frame
	(grabbed, frame) = camera.read()
	frame = cv2.flip(frame,1)

	
	# resize the frame, blur it, and convert it to the HSV
	# color space
##	frame = imutils.resize(frame, width=600)
	# blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	# construct a mask for the color "yellowish", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = cv2.inRange(hsv, greenLower, greenUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)

	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)[-2]
	center = None

	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		# only proceed if the radius meets a minimum size
		if radius > 10:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(frame, (int(x), int(y)), int(radius),
				(0, 255, 0), 2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)

			#angle = draw_angle(center,frame)
			#calculate_pid(angle)
                        
	# update the points queue
	pts.appendleft(center)

	# loop over the set of tracked points
	for i in xrange(1, len(pts)):
		# if either of the tracked points are None, ignore
		# them
		if pts[i - 1] is None or pts[i] is None:
			continue

		# otherwise, compute the thickness of the line and
		# draw the connecting lines
		thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
		cv2.line(frame, pts[i - 1], pts[i], (255, 0, 255), thickness)

	# show the frame to our screen
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF

	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		break

# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()
#arduino.close()
