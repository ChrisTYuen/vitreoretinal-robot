import os
import torch
import numpy as np
import cv2
from tools import functions
import segmentation_models_pytorch as smp


input_dir = '/home/yuki/Documents/eye_surgery/image_processing/data_tip/dataset/confidencemap/'
output_dir = '/home/yuki/Documents/eye_surgery/image_processing/data_tip/dataset/confidencemap_cropped_256_256/'

RESULT_DIR = '/home/yuki/Documents/eye_surgery/image_processing/data_tip/result/confidencemap_resize_resnet18/'
best_model = torch.load(RESULT_DIR+'best_model.pth')
ENCODER = 'resnet18'
ENCODER_WEIGHTS = 'imagenet'
DEVICE = 'cuda'

preprocessing_fn = smp.encoders.get_preprocessing_fn(ENCODER, ENCODER_WEIGHTS)
preprocessing = functions.get_preprocessing(preprocessing_fn)

output_size = 256
predict_size = 512
margin = 50

ratio_predict = predict_size/output_size


def get_tip_position(img, predict_size):
    original_size = np.shape(img)[0]
    ratio = original_size/predict_size
    img = cv2.resize(img, (predict_size, predict_size))
    x_img = preprocessing(image=img)['image']
    x_tensor = torch.from_numpy(x_img).to(DEVICE).unsqueeze(0)
    pr_mask = best_model.predict(x_tensor) # torch.tensor
    pr_mask = np.transpose((pr_mask.squeeze().cpu().numpy()) * 255, (1, 2, 0))

    tip_instrument = np.array(np.unravel_index(np.argmax(pr_mask[:, :, 0]), pr_mask[:, :, 0].shape))
    point_instrument = np.array(np.unravel_index(np.argmax(pr_mask[:, :, 1]), pr_mask[:, :, 1].shape))
    tip_shadow = np.array(np.unravel_index(np.argmax(pr_mask[:, :, 2]), pr_mask[:, :, 2].shape))

    return tip_instrument*ratio, point_instrument*ratio, tip_shadow*ratio


def max_distance(point1, point2, point3, center):
    dis1 = np.linalg.norm(center-point1)
    dis2 = np.linalg.norm(center - point2)
    dis3 = np.linalg.norm(center - point3)
    distances = np.array([dis1, dis2, dis3])

    return distances.max()


train_val_test = 'train/'
train_val_test_mask = 'trainannot/'
raw_images_ids = os.listdir(input_dir + train_val_test)
k = 0
while k < len(raw_images_ids):
    print(k)
    image = cv2.imread(input_dir + train_val_test + str(k) + '.png')
    mask1 = cv2.imread(input_dir + train_val_test_mask + str(k) + '-1.png', 0)
    mask2 = cv2.imread(input_dir + train_val_test_mask + str(k) + '-2.png', 0)
    mask3 = cv2.imread(input_dir + train_val_test_mask + str(k) + '-3.png', 0)

    tip_instrument, point_instrument, tip_shadow = get_tip_position(image, predict_size)
    center = (tip_instrument+point_instrument+tip_shadow)/3

    max_dis = max_distance(tip_instrument, point_instrument, tip_shadow, center)
    if max_dis+margin <= output_size/2:
        print("not_resized")
        left_top = center - np.array([output_size / 2, output_size / 2])
        right_bottom = center + np.array([output_size / 2, output_size / 2])
        left_top = left_top.astype('uint64')
        right_bottom = right_bottom.astype('uint64')
        # cv2.circle(image, (left_top[1], left_top[0]), 10, (255, 0, 0), -1)
        # cv2.circle(image, (right_bottom[1], right_bottom[0]), 10, (255, 0, 0), -1)
        image = image[left_top[0]:right_bottom[0], left_top[1]:right_bottom[1]]
        mask1 = mask1[left_top[0]:right_bottom[0], left_top[1]:right_bottom[1]]
        mask2 = mask2[left_top[0]:right_bottom[0], left_top[1]:right_bottom[1]]
        mask3 = mask3[left_top[0]:right_bottom[0], left_top[1]:right_bottom[1]]
    else:
        print("resized")
        dis = max_dis+margin
        left_top = center - np.array([dis, dis])
        right_bottom = center + np.array([dis, dis])
        left_top = left_top.astype('uint64')
        right_bottom = right_bottom.astype('uint64')
        image = image[left_top[0]:right_bottom[0], left_top[1]:right_bottom[1]]
        mask1 = mask1[left_top[0]:right_bottom[0], left_top[1]:right_bottom[1]]
        mask2 = mask2[left_top[0]:right_bottom[0], left_top[1]:right_bottom[1]]
        mask3 = mask3[left_top[0]:right_bottom[0], left_top[1]:right_bottom[1]]
        image = cv2.resize(image, (output_size, output_size))
        mask1 = cv2.resize(mask1, (output_size, output_size))
        mask2 = cv2.resize(mask2, (output_size, output_size))
        mask3 = cv2.resize(mask3, (output_size, output_size))

    cv2.imwrite(output_dir + train_val_test + str(k) + '.png', image)
    cv2.imwrite(output_dir + train_val_test_mask + str(k) + '-1.png', mask1)
    cv2.imwrite(output_dir + train_val_test_mask + str(k) + '-2.png', mask2)
    cv2.imwrite(output_dir + train_val_test_mask + str(k) + '-3.png', mask3)
    k = k + 1
