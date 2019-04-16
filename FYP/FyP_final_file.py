from collections import deque
import numpy as np
import argparse
import imutils
import cv2
from Tkinter import *
import ttk
import serial.tools.list_ports
import serial
from PIL import Image, ImageTk
import freenect
import random
from math import sqrt, atan2, degrees
import struct

greenLower = (148, 100, 82)
greenUpper = (255, 255, 255)
pts = deque(maxlen=64)

# PID Parameters

kp = 2
ki = 0.5
kd = 0.3
error = 0
last_error = 0
sum_error = 0
anti_windup = 10000

# Graph parameters
start_point = 20
gx = start_point
gy = 200
lastgx = 0
lastgy = 0
resolution = 1
graph_height, graph_width = 400, 400
graph = np.ones((graph_height, graph_width, 3), np.uint8)
cv2.line(graph, (start_point, 0), (start_point, graph_height), (255, 0, 0), 2)
cv2.line(graph, (0, int(graph_height / 2)), (graph_width, int(graph_height / 2)), (255, 0, 0), 2)


def draw_graph(angle):
    global gx, gy, graph_height, graph_width, lastgy, lastgx, start_point, resolution, graph
    lastgy = gy
    lastgx = gx
    gy = (angle + 180) * (50 - 350) / (180 + 180) + 350
    # gy = abs(angle) + graph_height/2

    # gy = random.randint(graph_height/2-100,graph_height/2+1
    # gy = (angle - (-90))*(350 - 150)/ (90 + 90) + 150
    gy = int(gy)
    # print gy
    ##print "gy: ", gy
    cv2.line(graph, (gx, gy), (gx, lastgy), (0, 0, 255), 1)
    if not gx > graph_width:
        gx = gx + resolution
    else:

        graph = np.ones((graph_height, graph_width, 3), np.uint8)
        cv2.line(graph, (start_point, 0), (start_point, graph_height), (255, 0, 0), 2)
        cv2.line(graph, (0, int(graph_height / 2)), (graph_width, int(graph_height / 2)), (255, 0, 0), 2)
        gx = start_point


def sendtoarduino(speed_, direction_):
    print abs(speed_), direction_
    if arduino.isOpen():
        if direction_ == 'c':
            arduino.write(struct.pack(">BB", abs(speed_), 255))
        elif direction_ == 'a':
            arduino.write(struct.pack(">BB", abs(speed_), 0))


def calculate_angle(x1, y1, x2, y2):  # X1,Y1 = Top fixed point, #X2,Y2 = Ball Center point
    x3 = x1
    y3 = y2
    perp = sqrt((x3 - x2) ** 2 + (y3 - y2) ** 2)
    base = sqrt((x1 - x3) ** 2 + (y1 - y3) ** 2)
    cal_angle = atan2(perp, base)
    cal_angle = degrees(cal_angle)
    if x2 > x1:
        direction = 'a'
    elif x2 < x1:
        direction = 'c'
    else:
        direction = 'n
    # print cal_angle
    return cal_angle, direction


def calculate_pid(current_point, direction_):
    global kp, ki, kd, error, last_error, sum_error, anti_windup

    if direction_ == 'c':
        current_point = current_point * -1

    setpoint = 0
    last_error = error
    error = setpoint - current_point
    corr = kp * error + ki * sum_error + kd * (last_error - error)

    if sum_error > anti_windup:
        sum_error = 0
    else:
        sum_error = sum_error + error

    if corr > 255:
        corr = 255
    elif corr < -255:
        corr = -255

    print corr
    return int(corr)


def get_video():
    array, _ = freenect.sync_get_video()
    array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
    return array


def get_depth():
    array, _ = freenect.sync_get_depth(format=freenect.DEPTH_REGISTERED)
    array = array.astype(np.uint8)
    return array


def process(frame):
    frame = imutils.resize(frame, width=400)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    d = 0
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        if radius > 10:
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            a, b = center
            ballx.set("Ball X-Position: " + str(a) + " Pixels")
            bally.set("Ball Y-Position: " + str(b) + " Pixels")
            cv2.line(frame, (graph_height / 2, 0), (graph_height / 2, graph_height), (255, 0, 0), 2)
            cv2.line(frame, (graph_height / 2, 0), (a, b), (255, 0, 0), 2)
            cal_angle, direction_ = calculate_angle(graph_height / 2, 0, a, b)
            cal_angle = int(cal_angle)
            correction = calculate_pid(cal_angle, direction_)
            sendtoarduino(correction, direction_)
            depth_img = get_depth()
            depth_ball = depth_img[center[0], center[1]]
            ballz.set("Ball Z-Position: %s mm" % str(depth_ball))

            ball_angle.set("Ball Angle: %s" % cal_angle)
            if direction_ == 'c':
                direction_ = 'Clockwise'
                d = (-1 * (cal_angle))
            elif direction_ == 'a':
                direction_ = 'Counter-Clockwise'
                d = ((cal_angle))
            else:
                direction_ = "Nuetral"
                d = ((cal_angle))
            motor_direction.set("Motor Diecrtion: %s" % str(direction_))

    else:
        ballx.set("Ball X-Position: None")
        bally.set("Ball Y-Position: None")
        ballz.set("Ball Z-Position: None")
        ball_angle.set("Ball Angle: None")
        motor_direction.set("Motor Diecrtion: None")
        d = 0

    pts.appendleft(center)
    if isDrawTrack.get():
        for i in xrange(1, len(pts)):
            if pts[i - 1] is None or pts[i] is None:
                continue
            thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)
            cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
    return frame, d


isrunning = 0
cap = cv2.VideoCapture(0)


def StartStreaming():
    global isrunning, graph
    if isrunning == 0:
        isrunning = 1
        process_label.pack(side=RIGHT)
        p = str(ports.get())
        p = p.split("-")

        def show_frame():
            frame = get_video()  # Get the image from Kinect
            frame = cv2.flip(frame, 1)
            frame, d = process(frame)
            cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
            img = Image.fromarray(cv2image)
            imgtk = ImageTk.PhotoImage(image=img)
            process_label.imgtk = imgtk
            process_label.configure(image=imgtk)
            draw_graph(d)
            graph_ = cv2.cvtColor(graph, cv2.COLOR_BGR2RGBA)
            graph_ = Image.fromarray(graph_)
            graph_ = ImageTk.PhotoImage(image=graph_)
            graph_label.imgtk = graph_
            graph_label.configure(image=graph_)

            if isrunning == 1:
                process_label.after(5, show_frame)
                # graph_label.after(5, show_frame)

    show_frame()


def StopStreaming():
    global isrunning
    isrunning = 0
    process_label.pack_forget()


def on_comport_select(event=None):
    print "Comport Selected: ",


def StartSerial():
    myport = str(ports.get()).strip('(')
    myport = myport.strip(')')
    myport = myport.split(',')
    print myport[0]
    if arduino.isOpen() == False:
        arduino.baudrate = 9600
        arduino.port = myport[0].strip("'")
        arduino.open()


def StopSerial():
    if arduino.isOpened() == True:
        arduino.close()


def list_ports():
    s = serial.tools.list_ports.comports()
    ports = []
    for i in s:
        ports.append(str(i))
    ports.append("None")
    return ports


arduino = serial.Serial()
root = Tk()
root.title("Kinect-based Ball-Beam Balance System")
ballx = StringVar()
bally = StringVar()
ballz = StringVar()
ball_angle = StringVar()
motor_direction = StringVar()
connstatus = StringVar()
isDrawTrack = BooleanVar()

ball_angle.set("Ball Angle: None")
ballx.set("Ball X-Position: None")
bally.set("Ball Y-Position: None")
ballz.set("Ball Z-Position: None")
connstatus.set("Connection Status: Not Connected")
motor_direction.set("Motor Diecrtion: None")
isDrawTrack.set(False)

frame1 = LabelFrame(root, text="Options")
frame1.grid(row=0, column=0, sticky="nswe", padx=5, pady=5)
frame2 = LabelFrame(root, text="Camera Feed")
frame2.grid(row=0, column=2, sticky="nswe", padx=5, pady=5)
frame3 = LabelFrame(root, text="Range Selector")
frame3.grid(row=0, column=1, sticky="nswe", padx=5, pady=5)
frame4 = LabelFrame(root, text="PID Graph")
frame4.grid(row=0, column=3, sticky="nswe", padx=5, pady=5)
process_label = Label(frame2)
process_label.grid(row=0, column=0, sticky="nswe", padx=5, pady=5)

graph_label = Label(frame4)
graph_label.grid(row=0, column=0, sticky="nswe", padx=5, pady=5)

Button1 = Button(frame1, text="Start Tracking", command=StartStreaming)
Button1.grid(row=0, column=0, sticky="nswe", padx=5, pady=5)
Button2 = Button(frame1, text="Stop Tracking", command=StopStreaming)
Button2.grid(row=1, column=0, sticky="nswe", padx=5, pady=5)
Button3 = Button(frame1, text="Connect to Arduino", command=StartSerial)
Button3.grid(row=2, column=0, sticky="nswe", padx=5, pady=5)
Button4 = Button(frame1, text="Disconnect from\n Arduino", command=StopSerial)
Button4.grid(row=3, column=0, sticky="nswe", padx=5, pady=5)
drawtrackentry = Checkbutton(frame1, text="Draw Tracks", variable=isDrawTrack)
drawtrackentry.grid(row=4, column=0, sticky="w", padx=5, pady=5)
ports = ttk.Combobox(frame1, values=list_ports())
ports.grid(row=5, column=0, sticky="w", padx=5, pady=5)
ports.bind('<<ComboboxSelected>>', on_comport_select)

Label(frame3, textvariable=connstatus).grid(row=0, column=0, sticky="nswe", padx=5, pady=5)
Label(frame3, textvariable=ballx).grid(row=1, column=0, sticky="w", padx=5, pady=5)
Label(frame3, textvariable=bally).grid(row=2, column=0, sticky="w", padx=5, pady=5)
Label(frame3, textvariable=ballz).grid(row=3, column=0, sticky="w", padx=5, pady=5)
Label(frame3, textvariable=ball_angle).grid(row=4, column=0, sticky="w", padx=5, pady=5)
Label(frame3, textvariable=motor_direction).grid(row=5, column=0, sticky="w", padx=5, pady=5)

root.mainloop()
cap.release()
arduino.close()
'''
SERIALLY
BLUETOOTH
CONNECTION(ARDUINO)

int
speed_ = 0;
int
dir = 0;
void
setup()
{
    pinMode(13, OUTPUT);
Serial.begin(9600);
}
void
loop()
{
while (Serial.available() >= 2)
    {
        speed_ = Serial.read();
    dir = Serial.read();
    if (dir == 255){
    digitalWrite(13, HIGH);
    }
    else {
    digitalWrite(13, LOW);
    }
    }'''
