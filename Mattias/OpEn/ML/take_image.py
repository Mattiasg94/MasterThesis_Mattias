from cv2 import cv2



cap = cv2.VideoCapture(1)
k=0
while True:
    _,frame=cap.read()
    cv2.imshow('frame',frame)
    key = cv2.waitKey(1)
    if key == 27:
        break
    if k==100:
        cv2.imwrite('frame.png',frame)
        print('save frame')
    k+=1
cap.release()
cv2.destroyAllWindows()
