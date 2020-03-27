from keras.preprocessing.image import img_to_array
from keras.models import load_model
import numpy as np
import argparse
import imutils
import pickle
from cv2 import cv2
import os
import time

cap = cv2.VideoCapture(0)
model = load_model('model')
mlb = pickle.loads(open('labelbin', "rb").read())

def inperpolate(CLS,proba, lst_val):
    CLS=list(CLS)
    proba=list(proba)
    idx1=proba.index(lst_val[-1])
    idx2=proba.index(lst_val[-2])
    val1 = float(CLS[idx1].split('_')[-1])
    val2 = float(CLS[idx2].split('_')[-1])
    p1 = lst_val[-1]/(lst_val[-1]+lst_val[-2])
    p2 = lst_val[-2]/(lst_val[-1]+lst_val[-2])
    val = val1*p1+val2*p2
    return val

test=[]
while True:
    start=time.time()
    _,frame=cap.read()
    image = cv2.resize(frame, (96, 96))
    image = image.astype("float") / 255.0
    image = img_to_array(image)
    image = np.expand_dims(image, axis=0)
    proba = model.predict(image)[0]
    idxs = np.argsort(proba)[::-1][:2]
    (X, Y, THETA) = ([], [], [])
    for i, c in enumerate(mlb.classes_):
        if 'th' in c:
            THETA.append(proba[i])
        elif 'x' in c:
            X.append(proba[i])
        elif 'y' in c:
            Y.append(proba[i])
    THETA.sort()
    X.sort()
    Y.sort()
    th = inperpolate(mlb.classes_,proba, THETA)
    x = inperpolate(mlb.classes_,proba, X)
    y = inperpolate(mlb.classes_,proba, Y)
    # for i,est in enumerate([x,y,th]):
    #     cv2.putText(frame, str(round(est,1)), (10, (i * 30) + 25),
    #     cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    # cv2.imshow('frame',frame)
    key = cv2.waitKey(1)
    if key == 27:
        break
    test.append(time.time()-start)
    if len(test)==500:
        break
print(sum(test)/len(test))
cap.release()
cv2.destroyAllWindows()



