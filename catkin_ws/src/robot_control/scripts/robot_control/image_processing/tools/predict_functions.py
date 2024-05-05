import matplotlib.pyplot as plt
import albumentations as albu
import cv2
import numpy as np


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


def get_instrument_tip(image,x_last,y_last,iteration):
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

#    gap = max_raw_low-max_raw_up
#    while gap > 2:
#        if max_col >= w-1 or max_raw_low < 1 or max_raw_up >= h-1:
#            break
#        max_col = max_col + 1
#        max_raw_up = max_raw_up + 1
#        max_raw_low = max_raw_low - 1
#        raw = 0
#        while raw < max_raw_low - max_raw_up + 1:
#            image[max_raw_up + raw, max_col] = 1
#            raw = raw + 1
        gap = max_raw_low-max_raw_up

    if iteration > 1:
        if np.sqrt((max_col-x_last)**2+(max_raw_low-y_last)**2)>10:
            max_col = x_last
            max_raw_low = y_last

    return max_col, max_raw_low

def get_instrument_other_end(image,x_last,y_last,iteration):
    h, w = image.shape
    max_col = 0
    max_raw = 0
    while np.sum(image[:, max_col]) < 1 and max_col < w-1:
        max_col = max_col + 1
    while image[max_raw, max_col] < 1 and max_raw < h-1:
        max_raw = max_raw + 1

    if iteration > 1:
        if np.sqrt((max_col-x_last)**2+(max_raw-y_last)**2)>15:
            max_col = x_last
            max_raw = y_last
    return max_col, max_raw


def get_shadow_tip(image,x_last,y_last,iteration):
    h, w = image.shape
#    max_col_right = w-1
#    max_raw = 0
#    max_col_left = 0
#    while np.sum(image[max_raw, :]) < 1 and max_raw < h-1:
#        max_raw = max_raw + 1
#    while image[max_raw, max_col_right] < 1 and max_col_right > 0:
#        max_col_right = max_col_right - 1
#    while image[max_raw, max_col_left] < 1 and max_col_left < w-1:
#        max_col_left = max_col_left + 1

#    if iteration > 1:
#        if np.sqrt((max_col_right-x_last)**2+(max_raw-y_last)**2)>25:
#            max_col_right = x_last
#            max_raw = y_last
#    return max_col_right, max_raw
    max_raw_up = 0
    max_raw_low = h-1
    max_col = w-1
    while np.sum(image[:, max_col]) < 1 and max_col > 0:
        max_col = max_col - 1
    while image[max_raw_low, max_col] < 1 and max_raw_low > 0:
        max_raw_low = max_raw_low - 1
    while image[max_raw_up, max_col] < 1 and max_raw_up < h-1:
        max_raw_up = max_raw_up + 1

    if iteration > 1:
        if np.sqrt((max_col-x_last)**2+(max_raw_up-y_last)**2)>10:
            max_col = x_last
            max_raw_up = y_last

    return max_col, max_raw_up




