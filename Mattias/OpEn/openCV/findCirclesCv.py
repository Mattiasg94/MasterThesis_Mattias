import numpy as np
from cv2 import cv2
num_clicks=3
begin=False
idx=-1
def nothing(x):
    pass
def mouse_drawing(event, x, y, flags, params):
    global xf1, yf1, xf2,yf2,idx,xr,yr,begin
    if event == cv2.EVENT_LBUTTONDOWN:
        idx =idx+1 if idx<(num_clicks-1) else 0
        if idx==0:
            (xf1,yf1)=(x,y)
        elif idx==1:
            (xf2,yf2)=(x,y)
        elif idx==2:
            (xr,yr)=(x,y)
            begin=True
            print('Begin')
        return
(c1X,c1Y,c2X,c2Y)=([],[],[],[])
alpha=0.8
def lpf(history, alpha):
    y = []
    yk = history[0]
    for k in range(len(history)):
        yk += alpha * (history[k]-yk)
        y.append(yk)
    return int(y[-1])

len_history=10
cap = cv2.VideoCapture(1);
init=True
cv2.namedWindow("Tracking")
cv2.createTrackbar("dp", "Tracking", 50, 100, nothing)
cv2.createTrackbar("param1", "Tracking", 20, 200, nothing)
cv2.createTrackbar("param2", "Tracking", 30, 200, nothing)
cv2.createTrackbar("minRadius", "Tracking", 7, 200, nothing)
cv2.createTrackbar("maxRadius", "Tracking", 25, 200, nothing)

cv2.namedWindow("output")
cv2.setMouseCallback("output", mouse_drawing)
while True:
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
        (X1,Y1,X2,Y2,D1,D2,R)=([],[],[],[],[],[],[])
        for i,(x, y ,r_detect) in enumerate(detected_circles[0, :]):
            if begin:
                if init:
                    init=False
                    d=np.sqrt((xf1-xf2)**2+(yf1-yf2)**2)*0.9
                    r=np.sqrt((xf2-xr)**2+(yf2-yr)**2)
                d1=np.sqrt((xf1-x)**2+(yf1-y)**2)
                d2=np.sqrt((xf2-x)**2+(yf2-y)**2)
                if d1<(r*4):
                    X1.append(x)
                    Y1.append(y)
                    D1.append(d1)
                if d2<(r*4):
                    X2.append(x)
                    Y2.append(y)
                    D2.append(d2)
                if len(D1)>0:
                    idx1=D1.index(min(D1))
                    x1k=X1[idx1]
                    y1k=Y1[idx1]
                    if len(c1X)>=len_history:
                        c1X.pop(0)
                        c1Y.pop(0)
                    c1X.append(x1k)
                    c1Y.append(y1k)
                    xf1=lpf(c1X,alpha)
                    yf1=lpf(c1Y,alpha)
                    # (xf1,yf1)=(int(sum(c1X)/len(c1X)),int(sum(c1Y)/len(c1Y)))
                    
                if len(D2)>0:
                    idx2=D2.index(min(D2))
                    x2k=X2[idx2]
                    y2k=Y2[idx2]
                    if len(c2X)>=len_history:
                        c2X.pop(0)
                        c2Y.pop(0)
                    c2X.append(x2k)
                    c2Y.append(y2k)    
                    xf2=lpf(c2X,alpha)
                    yf2=lpf(c2Y,alpha)
                    # (xf2,yf2)=(int(sum(c2X)/len(c2X)),int(sum(c2Y)/len(c2Y)))
                cv2.circle(output, (x, y), 2, (255, 0, 0), 3)
                cv2.circle(output, (x, y), r_detect, (255, 0, 0), 1)
                cv2.circle(output, (xf1, yf1), 2, (0, 255, 255), 3)
                cv2.circle(output, (xf2, yf2), 2, (0, 255, 255), 3)
            else:
                cv2.circle(output, (x, y), r_detect, (0, 0, 0), 1)
                cv2.circle(output, (x, y), 2, (0, 255, 255), 3)
    except Exception as e:
        print(e)
        pass
        
    color=(0,255,0)
    cv2.circle(output, (60,60), minRadius, color, thickness=1, lineType=8, shift=0)
    cv2.circle(output, (60,60), maxRadius, color, thickness=2, lineType=8, shift=0)
    cv2.imshow("output", output)

    key = cv2.waitKey(1)
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()