import os
import random
import csv
import numpy as np
from cv2 import cv2
from scipy import ndimage
from merge_image import overlay_image_alpha
from PIL import Image
# !-- SETUP---!
background_path = r'background.png'
car_path = r'car2.png'
# !-- SETUP END---!


size_w = 96
size_h = 96
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
car_img_resized = cv2.resize(
    car_img, (int(w_car_orig*resize_ratio_w), int(h_car_orig*resize_ratio_h)))
w_car = int(w_car_orig*resize_ratio_w)
h_car = int(h_car_orig*resize_ratio_h)
im_num = 0


background_pil = Image.fromarray(background)
car_img_pil = Image.fromarray(car_img)
background_pil.paste(car_img_pil, (xi,yi), car_img_pil)
img=np.array(background_pil) 

cv2.imwrite('big_test.jpg',img)
img = cv2.resize(img, (size_w, size_h))
cv2.imwrite('small_test.jpg',img)