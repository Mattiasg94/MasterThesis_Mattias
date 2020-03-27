# import the necessary packages
from keras.preprocessing.image import img_to_array
from keras.models import load_model
import numpy as np
import argparse
import imutils
import pickle
from cv2 import cv2
import os
from PIL import Image
from scipy import ndimage
from tensorflow import keras
# !-- SETUP---!
background_path = r'test_images/background.png'
car_path = r'test_images/car4.png'
x_true =88
y_true = 88
th_true = 15
classify_pos = False
classify_theta = not classify_pos
# !-- SETUP END---!


size_w = 76
size_h = 76
size_w_merge = size_w*3
size_h_merge = size_h*3
background_orig = cv2.imread(background_path)
w_back_orig = background_orig.shape[1]
h_back_orig = background_orig.shape[0]
background = cv2.resize(background_orig, (size_w_merge, size_w_merge))
resize_ratio_w = size_w_merge/w_back_orig
resize_ratio_h = size_w_merge/h_back_orig

car_img = cv2.imread(car_path, cv2.IMREAD_UNCHANGED)
w_car_orig = car_img.shape[1]
h_car_orig = car_img.shape[0]
car_img = ndimage.rotate(car_img, th_true)
car_img_resized = cv2.resize(
    car_img, (int(w_car_orig*resize_ratio_w), int(h_car_orig*resize_ratio_h)))
w_car = int(w_car_orig*resize_ratio_w)
h_car = int(h_car_orig*resize_ratio_h)
background_pil = Image.fromarray(background)
car_img_pil = Image.fromarray(car_img_resized)
background_pil.paste(car_img_pil, (x_true, y_true), car_img_pil)
image = np.array(background_pil)
hejpa = image
image = cv2.resize(image, (size_w, size_h))
output = image


def inperpolate(CLS, proba, lst_val):
    CLS = list(CLS)
    proba = list(proba)
    idx1 = proba.index(lst_val[-1])
    idx2 = proba.index(lst_val[-2])
    val1 = float(CLS[idx1].split('_')[-1])
    val2 = float(CLS[idx2].split('_')[-1])
    if lst_val[-2] > 0.7 and False:
        p1 = lst_val[-1]/(lst_val[-1]+lst_val[-2])
        p2 = lst_val[-2]/(lst_val[-1]+lst_val[-2])
        val = val1*p1+val2*p2
        return val, lst_val[-1]
    else:
        return val1, lst_val[-1]


if classify_pos:
    model_pos = load_model('model_pos')
    mlb_pos = pickle.loads(open('labelbin_pos', "rb").read())

    image = cv2.resize(image, (size_w, size_h))
    cv2.imshow('image',image)
    cv2.waitKey(0)
    image = image.astype("float") / 255.0
    image = img_to_array(image)
    image = np.expand_dims(image, axis=0)

    proba = model_pos.predict(image)[0]
    (X, Y) = ([], [])
    for i, c in enumerate(mlb_pos.classes_):
        if 'x' in c:
            X.append(proba[i])
        elif 'y' in c:
            Y.append(proba[i])
        if (proba[i]*100) > 10:
            print(mlb_pos.classes_[i], proba[i]*100)
    X.sort()
    Y.sort()
    # image = hejpa[int(y-h_car_orig*0.2):int((y+h_car_orig*1.2)),
    #               int(x-w_car_orig*0.2):int((x+w_car_orig*1.2))]

    x, xp = inperpolate(mlb_pos.classes_, proba, X)
    y, yp = inperpolate(mlb_pos.classes_, proba, Y)
    # THETA
    print('--------[VALUES]--------')
    print('Vals:',round(x, 2), round(y, 2))
    print('prob',round(xp, 2), round(yp, 2))
    print('Eror:',round(abs(x-x_true), 2), round(abs(y-y_true), 2))
    
def custom_loss(y_true, y_pred):
    return keras.backend.mean(keras.backend.square(y_true - y_pred), axis=-1)

if classify_theta:
    # Here comes the bug (no bug)
    model_theta = load_model('model_theta', compile=False)

    # model_theta = load_model('model_theta')
    mlb_theta = pickle.loads(open('labelbin_theta', "rb").read())
    image = cv2.resize(image, (int(w_car*1.2), int(h_car*1.2)))
    cv2.imwrite('image.jpg', image)
    #image = cv2.imread(r'theta_images/00053.png')
    image = hejpa[int(y_true-h_car*0.2):int((y_true+h_car*1.2+h_car*0.2)), int(x_true-w_car*0.2):int((x_true+w_car*1.2+w_car*0.2))]
    image = cv2.resize(image, (25, 34))
    image = image.astype("float") / 255.0
    image = img_to_array(image)
    image = np.expand_dims(image, axis=0)
    THETA = []
    proba_theta = model_theta.predict(image)[0]
    for i, c in enumerate(mlb_theta.classes_):
        THETA.append(proba_theta[i])
        if (proba_theta[i]*100) > 10:
            pass
        print(mlb_theta.classes_[i], proba_theta[i]*100)
    THETA.sort()
    th, thp = inperpolate(mlb_theta.classes_, proba_theta, THETA)

    print('--------[VALUES]--------')
    print(round(th, 2))
    print(round(thp, 2))

# show the output image
# cv2.imshow("Output", output)
# cv2.waitKey(0)
