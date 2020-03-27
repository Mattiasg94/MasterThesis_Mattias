import os
import random
import csv
import numpy as np
from cv2 import cv2
from scipy import ndimage
from PIL import Image
from imutils import paths

# !-- SETUP---!
background_path = r'test_images/background.png'
car_path = 'test_images/car2.png'
remove_only = False
make_pos_images = False
make_theta_images = not make_pos_images
# !-- SETUP END---!
DATASET_FOLDER = r"images\\"
DATASET_FOLDER_THETA = r'theta_images/'
POS_CVS_FILE = r"pos_annots.csv"
THETA_CVS_FILE = r"theta_annots.csv"
theta_back_paths_pos = sorted(list(paths.list_images(r'pos_backgrounds/')))
theta_back_paths = sorted(list(paths.list_images(r'theta_backgrounds/')))

size_w = 112
size_h = 112
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
    car_img, (int(w_car_orig*resize_ratio_w), int(h_car_orig*resize_ratio_h)), interpolation=cv2.INTER_AREA)
w_car = int(w_car_orig*resize_ratio_w)
h_car = int(h_car_orig*resize_ratio_h)
im_num = 0


filelist = [f for f in os.listdir(DATASET_FOLDER) if f.endswith(".png")]
for f in filelist:
    os.remove(os.path.join(DATASET_FOLDER, f))
filelist = [f for f in os.listdir(DATASET_FOLDER_THETA) if f.endswith(".png")]
for f in filelist:
    os.remove(os.path.join(DATASET_FOLDER_THETA, f))

if not remove_only and make_pos_images:
    with open(POS_CVS_FILE, "w") as pos_annot:
        writer = csv.writer(pos_annot, delimiter=",")
        for a in range(5):
            for i, xi in enumerate(range(0, size_w_merge-w_car+1, int(size_w_merge/10))):
                for j, yi in enumerate(range(0, size_h_merge-h_car+1, int(size_h_merge/10))):
                    for background_path in theta_back_paths_pos:
                        background=cv2.imread(background_path)
                        background = cv2.resize(background, (size_w_merge, size_w_merge))
                        rand_rotate = np.random.randint(0, 359)
                        car_img = ndimage.rotate(car_img_resized, rand_rotate)
                        rand_int = np.random.randint(8, 12)/10
                        noise_w_car = int(w_car*rand_int)
                        noise_h_car = int(h_car*rand_int)
                        car_img = cv2.resize(
                            car_img, (noise_w_car, noise_h_car), interpolation=cv2.INTER_AREA)
                        background_pil = Image.fromarray(background)
                        car_img_pil = Image.fromarray(car_img)
                        background_pil.paste(car_img_pil, (xi, yi), car_img_pil)
                        img = np.array(background_pil)
                        num = '0'*(5-len(str(im_num)))+str(im_num)
                        xnum = '0'*(4-len(str(xi)))
                        ynum = '0'*(4-len(str(yi)))
                        path = DATASET_FOLDER+num+".png"
                        row = [path, 'x_'+xnum+'_' +
                            str(xi), 'y_'+ynum+'_'+str(yi)]  # , 'th_'+thnum+'_'+str(thi)
                        writer.writerow(row)
                        img = cv2.resize(img, (size_w, size_h))
                        cv2.imwrite(path, img)
                        im_num += 1
    print('[Num Pos Images]:', im_num)

rand_f=[0.2]*4
if not remove_only and make_theta_images:
    with open(THETA_CVS_FILE, "w") as theta_annotation:
        writer = csv.writer(theta_annotation, delimiter=",")
        for j in range(1):
            for i, thi in enumerate(range(0, 360, 60)):
                for background_path in theta_back_paths:
                    background=cv2.imread(background_path)
                    background = cv2.resize(background, (size_w_merge, size_w_merge))
                    car_img = ndimage.rotate(car_img_resized, thi)
                    rand_int = np.random.randint(8, 12)/10
                    noise_w_car = int(w_car*rand_int)
                    noise_h_car = int(h_car*rand_int)
                    car_img = cv2.resize(
                        car_img, (noise_w_car, noise_h_car), interpolation=cv2.INTER_AREA)
                    background_pil = Image.fromarray(background)
                    car_img_pil = Image.fromarray(car_img)
                    rand_x = np.random.randint(w_car*2, size_w_merge-w_car*2)
                    rand_y = np.random.randint(h_car*2, size_h_merge-h_car*2)
                    background_pil.paste(car_img_pil, (rand_x, rand_y), car_img_pil)
                    img = np.array(background_pil)
                    ymin=int(rand_y-h_car*rand_f[0])
                    xmin=int(rand_x-w_car*rand_f[1])
                    ymax=int(rand_y+noise_h_car+h_car*rand_f[2])
                    xmax=int(rand_x+noise_w_car+w_car*rand_f[3])
                    img = img[ymin:ymax,xmin:xmax]
                    num = '0'*(5-len(str(im_num)))+str(im_num)
                    thnum = '0'*(4-len(str(thi)))
                    path = DATASET_FOLDER_THETA+num+".png"
                    row = [path,'th_'+thnum+'_'+str(thi)]
                    writer.writerow(row)
                    img = cv2.resize(img, (int(w_car*1.2), int(h_car*1.2)))
                    cv2.imwrite(path, img)
                    im_num += 1
                    rand_f.pop(0)
                    rand_f.append(np.random.randint(1,40)/100)
    print('[Num Theta Images]:', im_num)
