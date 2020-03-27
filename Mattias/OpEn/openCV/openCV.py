from cv2 import cv2
import numpy as np
from openCV_pos import get_angle_and_pos
import time


def nothing(x):
    pass

# init


(H_lst,S_lst,V_lst)=([],[],[])
l_b=np.array([0,0,0])
u_b=np.array([255,255,255])
(LB,UB,track_LB,track_UB)=([],[],[(0,0,0),(0,0,0)],[(255,255,255),(255,255,255)])
LB.append(l_b)
UB.append(u_b)
l_b=np.array([0,0,0])
u_b=np.array([255,255,255])
LB.append(l_b)
UB.append(u_b)

frame=[0]
images=['blue','red']
drawing = False
done_drawing=False
point1 = ()
point2 = ()
start=0
num_circles=2
idx=-1
def get_res_of_moving_obj():
    images=[]
    for i in range(2):
        l_b=LB[i]
        u_b=UB[i]
        l_b=track_LB[i]
        u_b=track_UB[i]
        mask = cv2.inRange(hsv, l_b, u_b)
        res = cv2.bitwise_and(frame, frame, mask=mask)
        images.append(res)
    return images

def mouse_drawing(event, x, y, flags, params):
    global point1, point2, drawing,done_drawing,start,idx
    if event == cv2.EVENT_RBUTTONDOWN:
        idx =idx+1 if idx<(num_circles-1) else 0
        # LB[idx]=track_LB
        # UB[idx]=track_UB
        return
    if event == cv2.EVENT_LBUTTONDOWN:
        if drawing is False:
            done_drawing=False
            drawing = True
            point1 = (x, y)
        else:
            drawing = False
            done_drawing=True
            start=time.time()
    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing is True:
            point2 = (x, y)
            
cv2.namedWindow("Tracking")
cv2.createTrackbar("LH", "Tracking", 0, 255, nothing)
cv2.createTrackbar("LS", "Tracking", 0, 255, nothing)
cv2.createTrackbar("LV", "Tracking", 0, 255, nothing)
cv2.createTrackbar("UH", "Tracking", 255, 255, nothing)
cv2.createTrackbar("US", "Tracking", 255, 255, nothing)
cv2.createTrackbar("UV", "Tracking", 255, 255, nothing)

cap = cv2.VideoCapture(1)
draw=True
if draw:
    cv2.namedWindow("CALIBRATE")
    cv2.setMouseCallback("CALIBRATE", mouse_drawing)
    circle=[0]
while True:
    _, frame = cap.read()
    frame= cv2.resize(frame,(500,400))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    l_h = cv2.getTrackbarPos("LH", "Tracking")
    l_s = cv2.getTrackbarPos("LS", "Tracking")
    l_v = cv2.getTrackbarPos("LV", "Tracking")

    u_h = cv2.getTrackbarPos("UH", "Tracking")
    u_s = cv2.getTrackbarPos("US", "Tracking")
    u_v = cv2.getTrackbarPos("UV", "Tracking")
    if (idx+1)!=num_circles:
        track_LB[idx]=np.array([l_h,l_s,l_v])
        track_UB[idx]=np.array([u_h,u_s,u_v])
    images =get_res_of_moving_obj()
    # cv2.imshow('blue',images[0])
    # cv2.imshow('red',images[1])
    
    xs,ys,angle,orig_img=get_angle_and_pos(frame,[images[0],images[1]])
    # cv2.imshow("frame", orig_img)
    subplot = cv2.hconcat([images[0],images[1],orig_img],)
    cv2.imshow('subplot',subplot)
    if draw:
        if point1 and point2:
            radius=int(np.sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2))
            color=(0,255,0)
            frame_masked=hsv.copy()
            circle=cv2.circle(hsv, point1, radius, color, thickness=-1, lineType=8, shift=0)
            mask = cv2.inRange(circle, (0,255,0), (0,255,0))
            frame_masked = cv2.bitwise_and(frame_masked, frame_masked, mask=mask)
            if done_drawing:
                npC=np.array(frame_masked)
                red_array=npC[:,:,0]
                green_array=npC[:,:,1]
                blue_array=npC[:,:,2]
                red_len=len(red_array[np.nonzero(red_array)])
                green_len=len(green_array[np.nonzero(green_array)])
                blue_len=len(blue_array[np.nonzero(blue_array)])
                H=np.sum(red_array)/red_len
                S=np.sum(green_array)/green_len
                V=np.sum(blue_array)/blue_len
        cv2.imshow("CALIBRATE", hsv)
        
    if done_drawing and time.time()-start<10:
        H_lst.append(H)
        S_lst.append(S)
        V_lst.append(V)
    elif done_drawing:
        Lower=0.8
        Upper=1.2
        lb=np.array([int(Lower*(sum(H_lst)/len(H_lst))),int(Lower*(sum(S_lst)/len(S_lst))),int(Lower*(sum(V_lst)/len(V_lst)))])
        ub=np.array([int(Upper*(sum(H_lst)/len(H_lst))),int(Upper*(sum(S_lst)/len(S_lst))),int(Upper*(sum(V_lst)/len(V_lst)))])
        cv2.setTrackbarPos("LH", "Tracking",lb[0])
        cv2.setTrackbarPos("LS", "Tracking",lb[1])
        cv2.setTrackbarPos("LV", "Tracking",lb[2])
        cv2.setTrackbarPos("UH", "Tracking",ub[0])
        cv2.setTrackbarPos("US", "Tracking",ub[1])
        cv2.setTrackbarPos("UV", "Tracking",ub[2])
        done_drawing=False

    key = cv2.waitKey(1)
    if key == 27:
        break
    time.sleep(0.1)
cv2.destroyAllWindows()