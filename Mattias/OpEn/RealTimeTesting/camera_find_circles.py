import numpy as np
from cv2 import cv2
import sys 
import os
import time
alpha=1

def nothing(x):
    pass

def mouse_drawing(event, x, y, flags, params):
    global xf1_last,yf1_last,xf2_last,yf2_last,xf1,yf1,xf2,yf2,r1,r2,idx,xs,ys
    global calibrate1,calibrate2,saftey_circle,saftey_circle_x,saftey_circle_y,begin,real_x,real_y,pixels_1M
    if event == cv2.EVENT_LBUTTONDOWN:
        idx =idx+1 if idx<(num_clicks-1) else 0
        if idx==0:
            (xf1_last,yf1_last)=(x,y)
            (xf1,yf1)=(x,y)
            (xs,ys)=(x,y)
        elif idx==1:
            r1=np.sqrt((x-xf1_last)**2+(y-yf1_last)**2)
            calibrate1=True
            print('calibrate1')
        elif idx==2:
            (xf2_last,yf2_last)=(x,y)
            (xf2,yf2)=(x,y)
        elif idx==3:
            r2=np.sqrt((x-xf2_last)**2+(y-yf2_last)**2)
            print('calibrate2')
            calibrate2=True
        elif idx==4:
            (saftey_circle_x,saftey_circle_y)=(x,y)
        elif idx==5:
            saftey_circle=np.sqrt((x-saftey_circle_x)**2+(y-saftey_circle_y)**2)
            print('Begin with 1 m calc')
        elif idx==6:
            (real_x,real_y)=(x,y)
        elif idx==7:
            pixels_1M=np.sqrt((x-real_x)**2+(y-real_y)**2)
            print('Begin')
            begin=True
        return

num_clicks=8
(begin,calibrate1,calibrate2)=(False,False,False)
idx=-1
init=True
cv2.namedWindow("Tracking")
cv2.createTrackbar("dp", "Tracking", 60, 100, nothing)
cv2.createTrackbar("param1", "Tracking", 25, 200, nothing)
cv2.createTrackbar("param2", "Tracking", 21, 200, nothing)
cv2.createTrackbar("minRadius", "Tracking", 6, 200, nothing)
cv2.createTrackbar("maxRadius", "Tracking", 23, 200, nothing)

cv2.namedWindow("output")
cv2.setMouseCallback("output", mouse_drawing)

R=[]

def get_angle_and_pos(x1,x2,y1,y2):
    angle=np.arctan2(y1-y2,x1-x2)
    d=np.sqrt(np.power(x1-x2,2)+np.power(y1-y2,2))
    xs = x1 - np.cos(angle)*d/2
    ys = y1 - np.sin(angle)*d/2
    y1=800-y1
    y2=800-y2
    angle=np.arctan2(y1-y2,x1-x2)
    return xs,ys,angle

xs=ys=0
pixels_1M=1
camera_is_ready=False
def get_camera_measurements(cap):
    angle=0
    global cam_hight,pixels_1M,camera_is_ready,R,xf1_last,yf1_last,xf2_last,yf2_last,xf1,yf1,xf2,yf2,r1,r2,idx,calibrate1,calibrate2,saftey_circle,saftey_circle_x,saftey_circle_y,begin,xs,ys
    l_h = cv2.getTrackbarPos("LH", "Tracking")
    dp=cv2.getTrackbarPos("dp", "Tracking")
    param1=cv2.getTrackbarPos("param1", "Tracking")
    param2=cv2.getTrackbarPos("param2", "Tracking")
    minRadius=cv2.getTrackbarPos("minRadius", "Tracking")
    maxRadius=cv2.getTrackbarPos("maxRadius", "Tracking")
    _, frame = cap.read()
    output = frame.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 5)
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, dp,
                            param1=param1, param2=param2, minRadius=minRadius, maxRadius=maxRadius)
    try:
        detected_circles = np.uint16(np.around(circles))
        for (x, y ,r_detect) in detected_circles[0, :]:
            if begin:
                detect_b=detect_s=False
                d=np.sqrt((x-xs)**2+(y-ys)**2)
                if not d>saftey_circle and not detect_b:
                    if r_detect>=r1*1.2 and r_detect<=r2*1.2: # add to big
                        xf2+= alpha * (x-xf2_last)
                        xf2_last=xf2
                        yf2+=alpha * (y-yf2_last)
                        yf2_last=yf2
                        detect_b=True
                    if r_detect<=r1*1.2 and not detect_s: # add to small
                        xf1+= alpha * (x-xf1_last)
                        xf1_last=xf1
                        yf1+=alpha * (y-yf1_last)
                        yf1_last=yf1
                        detect_s=True         
                    cv2.circle(output, (int(xf1), int(yf1)), 2, (0, 255, 0), 3)
                    cv2.circle(output, (int(xf2), int(yf2)), 2, (0, 0,255), 3)
                    xs,ys,angle=get_angle_and_pos(xf1,xf2,yf1,yf2)
                    camera_is_ready=True
                    if detect_b and detect_s:
                        break
            else:
                if calibrate1:
                    d=np.sqrt((x-xf1)**2+(y-yf1)**2)
                    if d<r1*1.5:                        
                        R.append(r_detect)
                    if len(R)>40:
                        r1=sum(R)/len(R)
                        print('Done calibrating Circle 1')
                        calibrate1=False
                        R=[]
                elif calibrate2:
                    d=np.sqrt((x-xf2)**2+(y-yf2)**2)
                    if d<r2*1.5:                        
                        R.append(r_detect)
                    if len(R)>40:
                        r2=sum(R)/len(R)
                        print('Done calibrating Circle 2')
                        calibrate2=False                        
                        print('Start with satfy distance')
                cv2.circle(output, (x, y), r_detect, (0, 0, 0), 1)
                cv2.circle(output, (x, y), 2, (0, 255, 255), 3)
    except Exception as e:
        exc_type, exc_obj, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        if not 'oop of ufunc does not sup' in str(e):
            print(exc_type, fname, exc_tb.tb_lineno,e)
        
    color=(0,255,0)
    cv2.circle(output, (60,60), minRadius, color, thickness=1, lineType=8, shift=0)
    cv2.circle(output, (60,60), maxRadius, color, thickness=2, lineType=8, shift=0)
    return xs,ys,angle,output,camera_is_ready,pixels_1M
