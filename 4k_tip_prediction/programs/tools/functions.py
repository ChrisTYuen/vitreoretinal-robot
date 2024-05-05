import matplotlib.pyplot as plt
import albumentations as albu
import numpy as np
import cv2
import shutil
from labelme import utils
from scipy.ndimage.filters import minimum_filter
import json
import torch
import os


def visualize(**images):
    """PLot images in one row."""
    n = len(images)
    plt.figure(figsize=(16, 5))
    for i, (name, image) in enumerate(images.items()):
        plt.subplot(1, n, i + 1)
        plt.xticks([])
        plt.yticks([])
        plt.title(' '.join(name.split('_')).title())
        plt.imshow(image)
    plt.show()


def get_gaussian_confidence_map(w, h, tip_x, tip_y, sigma):
    width = np.arange(0, w, 1)
    height = np.arange(0, h, 1)
    X, Y = np.meshgrid(width, height)
    mu = np.array([tip_x, tip_y])
    S = np.array([[sigma, 0], [0, sigma]])

    x_norm = (np.array([X, Y]) - mu[:, None, None]).transpose(1, 2, 0)
    heatmap = np.exp(- x_norm[:, :, None, :] @ np.linalg.inv(S)[None, None, :, :] @ x_norm[:, :, :, None] / 2.0) * 255

    return heatmap[:, :, 0, 0]


def make_mask_image_confidence_map(w, h, x, y, sigma, path):
    img = get_gaussian_confidence_map(w, h, x, y, sigma)
    cv2.imwrite(path, img)
    return img


def get_instrument_tip_from_json(image_path):
    with open(image_path, "r", encoding="utf-8") as f:
        dj = json.load(f)
    if dj['shapes'][0]['label'] == 'tip_instrument':
        points = dj['shapes'][0]['points']
    elif dj['shapes'][1]['label'] == 'tip_instrument':
        points = dj['shapes'][1]['points']
    else:
        points = dj['shapes'][2]['points']
    xy = [tuple(point) for point in points]
    return xy[0]


def get_shadow_tip_from_json(image_path):
    with open(image_path, "r", encoding="utf-8") as f:
        dj = json.load(f)

    if dj['shapes'][0]['label'] == 'tip_shadow':
        points = dj['shapes'][0]['points']
    elif dj['shapes'][1]['label'] == 'tip_shadow':
        points = dj['shapes'][1]['points']
    else:
        points = dj['shapes'][2]['points']
    xy = [tuple(point) for point in points]
    return xy[0]


def get_another_instrument_point_from_json(image_path):
    with open(image_path, "r", encoding="utf-8") as f:
        dj = json.load(f)

    if dj['shapes'][0]['label'] == 'point_instrument':
        points = dj['shapes'][0]['points']
    elif dj['shapes'][1]['label'] == 'point_instrument':
        points = dj['shapes'][1]['points']
    else:
        points = dj['shapes'][2]['points']
    xy = [tuple(point) for point in points]
    return xy[0]


def max_distance(point1, point2, point3, center):
    dis1 = np.linalg.norm(center - point1)
    dis2 = np.linalg.norm(center - point2)
    # dis3 = np.linalg.norm(center - point3)
    # distances = np.array([dis1, dis2, dis3])
    distances = np.array([dis1, dis2])

    return distances.max()


def get_image_size(image_path):
    with open(image_path, "r", encoding="utf-8") as f:
        dj = json.load(f)
    w = dj['imageWidth']
    h = dj['imageHeight']
    return w, h


def get_augmentation():
    train_transform = [
        albu.Blur(), # blur_limit=21, p=1
        albu.MotionBlur(), # blur_limit=21
        albu.GaussianBlur(), # blur_limit=21
        albu.GlassBlur(),
        # albu.GaussNoise(p=0.2),
        # albu.ImageCompression(),
        albu.ISONoise(),
        albu.MultiplicativeNoise(),
        albu.Downscale(),
        # albu.Rotate(),
        # albu.OpticalDistortion(),
        # albu.GridDistortion(),
        # albu.ElasticTransform(),
        # albu.RandomGridShuffle(),
        # albu.HueSaturationValue(),
        # albu.RGBShift(),
        albu.ColorJitter(),
        # albu.ChannelDropout(),
        # albu.Normalize(),
        albu.RandomGamma(),
        # albu.RandomBrightness(),
        # albu.RandomContrast(),
        albu.RandomBrightnessContrast(),
        # albu.Compose(
        #     [
        #         albu.Rotate(),
        #         albu.OneOf(
        #             [
        #                 albu.Blur(blur_limit=21, p=1),
        #                 albu.MotionBlur(blur_limit=21),
        #                 albu.GaussianBlur(blur_limit=21),
        #                 albu.GlassBlur(),
        #                 albu.GaussNoise(p=0.2),
        #                 albu.ImageCompression(),
        #                 albu.ISONoise(),
        #                 albu.MultiplicativeNoise(),
        #                 albu.Downscale(),
        #                 albu.OpticalDistortion(),
        #                 albu.GridDistortion(),
        #                 albu.ElasticTransform(),
        #                 # albu.RandomGridShuffle(),
        #                 # albu.HueSaturationValue(),
        #                 # albu.RGBShift(),
        #                 # albu.ChannelDropout(),
        #                 albu.Normalize(),
        #                 albu.RandomGamma(),
        #                 albu.RandomBrightness(),
        #                 albu.RandomContrast(),
        #                 albu.RandomBrightnessContrast(),
        #             ]
        #         )
        #     ]
        # ),
    ]
    return albu.OneOf(train_transform)


def save_augmented_images(save_raw_dir, save_mask_dir, raw_image, mask_image, iteration, first_num):
    raw_output_path = save_raw_dir + str(iteration + first_num) + '.png'
    cv2.imwrite(raw_output_path, raw_image)

    mask_output_path1 = save_mask_dir + str(iteration + first_num) + '-1.png'
    cv2.imwrite(mask_output_path1, mask_image[:, :, 0])
    mask_output_path2 = save_mask_dir + str(iteration + first_num) + '-2.png'
    cv2.imwrite(mask_output_path2, mask_image[:, :, 1])
    mask_output_path3 = save_mask_dir + str(iteration + first_num) + '-3.png'
    cv2.imwrite(mask_output_path3, mask_image[:, :, 2])


def get_images(input_raw_dir, input_mask_dir, total_image_num):
    n = np.random.choice(total_image_num - 1) + 1
    raw = cv2.imread(input_raw_dir + str(n) + '.png')
    shape = np.shape(raw)
    # raw = cv2.cvtColor(raw, cv2.COLOR_BGR2RGB)
    mask = np.zeros([shape[0], shape[1], shape[2]])
    mask[:, :, 0] = cv2.imread(input_mask_dir + str(n) + '-1.png', 0)
    mask[:, :, 1] = cv2.imread(input_mask_dir + str(n) + '-2.png', 0)
    mask[:, :, 2] = cv2.imread(input_mask_dir + str(n) + '-3.png', 0)
    return raw, mask


def copy_files(get_raw_dir, get_mask_dir, output_raw_dir, output_mask_dir, input_iteration, output_iteration):
    shutil.copyfile(get_raw_dir + str(input_iteration) + '.png', output_raw_dir + str(output_iteration) + '.png')
    shutil.copyfile(get_mask_dir + str(input_iteration) + '-1.png', output_mask_dir + str(output_iteration) + '-1.png')
    shutil.copyfile(get_mask_dir + str(input_iteration) + '-2.png', output_mask_dir + str(output_iteration) + '-2.png')
    shutil.copyfile(get_mask_dir + str(input_iteration) + '-3.png', output_mask_dir + str(output_iteration) + '-3.png')
    
  
def visualize(**images):
    """PLot images in one row."""
    n = len(images)
    plt.figure(figsize=(16, 5))
    for i, (name, image) in enumerate(images.items()):
        plt.subplot(1, n, i + 1)
        plt.xticks([])
        plt.yticks([])
        plt.title(' '.join(name.split('_')).title())
        plt.imshow(image)
    plt.show()


def get_validation_augmentation():
    """Add paddings to make image shape divisible by 32"""
    test_transform = [
        # albu.PadIfNeeded(384, 480)
    ]
    return albu.Compose(test_transform)


def to_tensor(x, **kwargs):
    return x.transpose(2, 0, 1).astype('float32')


def get_preprocessing(preprocessing_fn):
    """Construct preprocessing transform

    Args:
        preprocessing_fn (callbale): data normalization function
            (can be specific for each pretrained neural network)
    Return:
        transform: albumentations.Compose

    """

    _transform = [
        albu.Lambda(image=preprocessing_fn),
        albu.Lambda(image=to_tensor, mask=to_tensor),
    ]
    return albu.Compose(_transform)


def get_instrument_tip(image):
    h, w = image.shape
    max_raw_up = 0
    max_raw_low = h-1
    max_col = w-1
    while np.sum(image[:, max_col]) < 1 and max_col > 0:
        max_col = max_col - 1
    while image[max_raw_low, max_col] < 1 and max_raw_low > 0:
        max_raw_low = max_raw_low - 1
    while image[max_raw_up, max_col] < 1 and max_raw_up < h-1:
        max_raw_up = max_raw_up + 1

    gap = max_raw_low-max_raw_up
    while gap > 2:
        if max_col >= w-1 or max_raw_low < 1 or max_raw_up >= h-1:
            break
        max_col = max_col + 1
        max_raw_up = max_raw_up + 1
        max_raw_low = max_raw_low - 1
        raw = 0
        while raw < max_raw_low - max_raw_up + 1:
            image[max_raw_up + raw, max_col] = 1
            raw = raw + 1
        gap = max_raw_low-max_raw_up

    return max_col, max_raw_low


def get_shadow_tip(image):
    h, w = image.shape
    max_col_right = w-1
    max_raw = 0
    max_col_left = 0
    while np.sum(image[max_raw, :]) < 1 and max_raw < h-1:
        max_raw = max_raw + 1
    while image[max_raw, max_col_right] < 1 and max_col_right > 0:
        max_col_right = max_col_right - 1
    while image[max_raw, max_col_left] < 1 and max_col_left < w-1:
        max_col_left = max_col_left + 1

    # gap = max_col_right - max_col_left
    # while gap > 0:
    #     if max_raw >= h - 1 or max_col_left < 1 or max_col_right >= w - 1:
    #         break
    #     max_raw = max_raw - 1
    #     max_col_right = max_col_right - 1
    #     max_col_left = max_col_left + 1
    #     image[max_raw, max_col_left] = 1
    #     image[max_raw, max_col_right] = 1
    #     gap = max_col_right - max_col_left
    return max_col_right, max_raw


def get_ROI_center(tip_instrument, point_instrument, tip_shadow):
    center = (tip_instrument + tip_shadow) / 2
    return center


def detect_peaks(image, filter_size=2, order=1):
    local_min = minimum_filter(image, footprint=np.ones((filter_size, filter_size)), mode='constant')
    detected_peaks = np.ma.array(image, mask=~(image == local_min))

    # 小さいピーク値を排除（最大ピーク値のorder倍のピークは排除）
    temp = np.ma.array(detected_peaks, mask=~(detected_peaks >= detected_peaks.max() * order))
    peaks_index = np.where((temp.mask != True))
    return peaks_index


def predict_and_mark(resized_img, left_top, best_model, preprocessing, parameters, ROI_dis, i):
    ratio = ROI_dis / parameters.predict_size * 2
    x_img = preprocessing(image=resized_img)['image']
    x_tensor = torch.from_numpy(x_img).to(parameters.DEVICE).unsqueeze(0)
    pr_mask = best_model.predict(x_tensor) # torch.tensor
    pr_mask = np.transpose((pr_mask.squeeze().cpu().numpy()) * 255, (1, 2, 0))

    # cv2.imshow('instrument', pr_mask[:, :, 0])
    # cv2.waitKey(1)
    # cv2.imshow('shadow', pr_mask[:, :, 1])
    # cv2.waitKey(1)
    # cv2.imshow('shaft', pr_mask[:, :, 2])
    # cv2.waitKey(1)
    RESULT_DIR = parameters.result_path
    SAVE_DIR = os.path.join(RESULT_DIR, 'video/check/')

    # if i % 50 == 0:
    #     cv2.imwrite(SAVE_DIR + str(i + 1) + '-test' + '.png', resized_img)
    #     # cv2.imwrite(SAVE_DIR + str(i + 1) + '-test_mask1' + '.png', mask_vis_test[:, :, 0] * 255)
    #     # cv2.imwrite(SAVE_DIR + str(i + 1) + '-test_mask2' + '.png', mask_vis_test[:, :, 1] * 255)
    #     # cv2.imwrite(SAVE_DIR + str(i + 1) + '-test_mask3' + '.png', mask_vis_test[:, :, 2] * 255)
    #
    #     cv2.imwrite(SAVE_DIR + str(i + 1) + '-test_predict1' + '.png', pr_mask[:, :, 0])
    #     cv2.imwrite(SAVE_DIR + str(i + 1) + '-test_predict2' + '.png', pr_mask[:, :, 1])
    #     cv2.imwrite(SAVE_DIR + str(i + 1) + '-test_predict3' + '.png', pr_mask[:, :, 2])

    tip_instrument = np.array(np.unravel_index(np.argmax(pr_mask[:, :, 0]), pr_mask[:, :, 0].shape))
    tip_shadow = np.array(np.unravel_index(np.argmax(pr_mask[:, :, 1]), pr_mask[:, :, 1].shape))
    point_instrument = 2*np.array(np.unravel_index(np.argmax(pr_mask[:, :, 2]), pr_mask[:, :, 2].shape))-tip_instrument
    center = get_ROI_center(tip_instrument, point_instrument, tip_shadow)

    max_dis = max_distance(tip_instrument, tip_shadow, point_instrument, center)*ratio

    cv2.line(resized_img, (point_instrument[1], point_instrument[0]), (tip_instrument[1], tip_instrument[0]),
             (255, 0, 0), 4)
    cv2.circle(resized_img, (tip_instrument[1], tip_instrument[0]), 5, (255, 0, 0), -1)
    cv2.circle(resized_img, (tip_shadow[1], tip_shadow[0]), 5, (0, 0, 255), -1)
    if parameters.print_center:
        cv2.circle(resized_img, (int(center[1]), int(center[0])), 10, (0, 255, 0), -1)

    vec_1 = point_instrument - tip_instrument
    vec_2 = tip_shadow - tip_instrument
    shaft_dis = np.linalg.norm(vec_1.dot(vec_2)/(vec_1.dot(vec_1))*vec_1-vec_2)*ratio
    tip_distance = int(np.linalg.norm(tip_instrument - tip_shadow) * ratio)
    cv2.putText(resized_img, 'tip_dis:' + str(tip_distance) + 'px',
                (int(parameters.predict_size*250/512), int(parameters.predict_size*420/512)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    if shaft_dis < 1000:
        cv2.putText(resized_img, 'shaft_dis:' + str(int(shaft_dis)) + 'px',
                    (int(parameters.predict_size*250/512), int(parameters.predict_size*450/512)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    cv2.putText(resized_img, 'ROI_size:' + str(int(ROI_dis*2)) + 'px',
                (int(parameters.predict_size*250/512), int(parameters.predict_size*480/512)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

    ROI_center = left_top + np.array([int(center[1]*ratio), int(center[0]*ratio)])
    return resized_img, ROI_center, max_dis
