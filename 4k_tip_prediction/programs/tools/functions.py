import matplotlib.pyplot as plt
import albumentations as albu
import numpy as np
import cv2
import shutil
from labelme import utils
from scipy.ndimage.filters import minimum_filter
from collections import deque
import json
import torch
import os

import sys
sys.path.append('/home/yuki/Documents/MyProjects/4k_tip_prediction/programs/tools')
from Parameters import Parameters

class ROIPredictor:
    """
    Class for predicting the region of interest (ROI) in the frame
    """
    def __init__(self, roi_model, preprocessing, parameters):
        self.roi_model = roi_model
        self.preprocessing = preprocessing
        self.parameters = parameters
        self.previous_centers = deque(maxlen=self.parameters.SMA_smoothing_N)
        self.freeze_counter = 0

    def resize_image(self, image, parameters, width, height):
        """
        Resize the image to the specified size and crop out the image if size is not divisible by the stride of 16
        """
        # Resize the image to the specified size    
        resize_height = parameters.ROI_resize_size
        resize_width = int((parameters.ROI_resize_size * width) / height)
        image = cv2.resize(image, (resize_width, parameters.ROI_resize_size))

        # Crop out image if size is not divisible by the stride of 16 
        crop_height = resize_height % 16
        crop_width = resize_width % 16
        crop_height_top = crop_height // 2
        crop_height_bottom = crop_height - crop_height_top
        crop_width_left = crop_width // 2
        crop_width_right = crop_width - crop_width_left

        image = image[crop_height_top:resize_height-crop_height_bottom, crop_width_left:resize_width-crop_width_right]
        new_width, new_height = image.shape[1], image.shape[0]
        return image, resize_width, resize_height, crop_width_left, crop_height_top, new_width, new_height

    @staticmethod
    def scale_keypoints(old_x, old_y, old_w, old_h, new_w, new_h, crop_w_l, crop_h_t, resize_w, resize_h):
        """
        Scale the keypoints to the original image size
        """
        new_x = old_x * (new_w / old_w)
        new_y = old_y * (new_h / old_h)
        adjusted_x = new_x + crop_w_l * new_w / resize_w 
        adjusted_y = new_y + crop_h_t * new_h / resize_h

        return adjusted_x, adjusted_y

    @staticmethod
    def get_ROI_center(tip_instrument, tip_shadow):
        """
        Calculate the center of the region of interest (ROI)
        """
        center = (tip_instrument + tip_shadow) / 2
        return center

    def predict_ROI(self, frame, original_w, original_h):
        """
        Predict the region of interest (ROI) in the frame and smooth the results using a simple moving average (SMA) filter
        """
        frame, resize_width, resize_height, crop_width_left, crop_height_top, new_width, new_height = self.resize_image(frame, self.parameters, original_w, original_h)
        x_img = self.preprocessing(image=frame)['image']
        x_tensor = torch.from_numpy(x_img).to(self.parameters.DEVICE).unsqueeze(0)
        pr_mask = self.roi_model.predict(x_tensor) # torch.tensor
        pr_mask = np.transpose((pr_mask.squeeze().cpu().numpy()) * 255, (1, 2, 0))

        # Unravel indecies of the maximum value in each channel and calculate center, max distance
        tip_instrument = np.array(np.unravel_index(np.argmax(pr_mask[:, :, 0]), pr_mask[:, :, 0].shape))
        tip_shadow = np.array(np.unravel_index(np.argmax(pr_mask[:, :, 1]), pr_mask[:, :, 1].shape))
        point_instr = np.array(np.unravel_index(np.argmax(pr_mask[:, :, 2]), pr_mask[:, :, 2].shape))
        center = self.get_ROI_center(tip_instrument, tip_shadow)
        max_dis = max_distance(tip_instrument, tip_shadow, center)
        
        # Adjust points to original image size considering cropped area
        (ROI_adjusted_x, ROI_adjusted_y
        ) = self.scale_keypoints(center[1], center[0], new_width, new_height, original_w, original_h, crop_width_left, crop_height_top, resize_width, resize_height)
        (point_inst_adjusted_x, point_inst_adjusted_y
        ) = self.scale_keypoints(point_instr[1], point_instr[0], new_width, new_height, original_w, original_h, crop_width_left, crop_height_top, resize_width, resize_height)
        ROI_center = np.array([ROI_adjusted_x, ROI_adjusted_y])
        point_instrument = np.array([point_inst_adjusted_x, point_inst_adjusted_y])

        # If there's a previous center and the change is too large, ignore this center
        if self.previous_centers and np.linalg.norm(ROI_center - self.previous_centers[-1]) > self.parameters.outlier_threshold_roi:
            self.freeze_counter += 1
            if self.freeze_counter >= self.parameters.freeze_threshold:
                self.freeze_counter = 0
                self.previous_centers.clear()
                return ROI_center, max_dis, point_instrument
            return self.previous_centers[-1], max_dis, point_instrument
        
        self.freeze_counter = 0 

        # Add the new center to the deque and compute the average
        self.previous_centers.append(ROI_center)
        smooth_ROI_center = np.mean(self.previous_centers, axis=0)

        return smooth_ROI_center, max_dis, point_instrument

class Detector:
    """
    Class for detecting the instrument tip, shadow, and shaft in the frame
    """
    def __init__(self, best_model, preprocessing, parameters):
        self.best_model = best_model
        self.preprocessing = preprocessing
        self.parameters = parameters
        self.previous_point_instrument = deque(maxlen=1)
        self.previous_point_shadow = deque(maxlen=self.parameters.EMA_shadow_N)
        self.previous_point_other = deque(maxlen=parameters.EMA_point_N)
        self.shaft_vector = None
        self.stuck_counter = 0

    @staticmethod
    def get_instrument_tip(image):
        """
        Get the instrument tip from the image
        """
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
    
    @staticmethod
    def get_shadow_tip(image):
        """
        Get the shadow tip from the image
        """
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
    
    @staticmethod
    def outlier_detection(previous_point, point, threshold, name):
        if previous_point and np.linalg.norm(point - previous_point[-1]) > threshold:
            print(name, 'outlier detected:', np.linalg.norm(point - previous_point[-1]))
            return previous_point[-1]
        else:
            return point
        
    def vector_ignore(self, current_vector):
        if self.shaft_vector is None:
            self.shaft_vector = current_vector
        # Calculate angle between the current vector and the previous vector
        dot_product = np.dot(current_vector, self.shaft_vector)
        norm_current = np.linalg.norm(current_vector)
        norm_previous = np.linalg.norm(self.shaft_vector)
        cos_angle = dot_product / (norm_current * norm_previous)
        angle = np.degrees(np.arccos(cos_angle))
        self.shaft_vector = current_vector
        if np.isnan(angle):
            angle = 0

        return angle
    
    @staticmethod
    def EMA(previous_point, alpha):
        """
        Get the Exponential Moving Average (EMA) of the points
        """
        if len(previous_point) == 1:   
            point = previous_point[-1]  # First data point is set as initial EMA
            return point
        else:                                          
            point = alpha  * previous_point[-1] + (1 - alpha) * np.mean(list(previous_point)[:-1])  # EMA calculation
            return point

    def predict_and_mark(self, ROI_dis, preprocessed_frame_interested, resized_img, point_instrument, i):
        """
        predict_and_mark(self, resized_img, header_time, ROI_center, ROI_half_size) predicts the positions of
        the instruments' tips and the shaft line of the surgical instrument, marks these values on the received image
        for showing, calculates the shaft distance and tip distance for autonomous positioning, and publishes
        the calculated distances.
        """
        ratio = ROI_dis / self.parameters.predict_size * 2
        x_img = self.preprocessing(image=preprocessed_frame_interested)['image']
        x_tensor = torch.from_numpy(x_img).to(self.parameters.DEVICE).unsqueeze(0)
        pr_mask = self.best_model.predict(x_tensor) # torch.tensor
        pr_mask = np.transpose((pr_mask.squeeze().cpu().numpy()) * 255, (1, 2, 0))
        if self.parameters.resize_input:
            pr_mask = cv2.resize(pr_mask, (self.parameters.output_size, self.parameters.output_size))

        # cv2.imshow('instrument', pr_mask[:, :, 0])
        # cv2.waitKey(1)
        # cv2.imshow('shadow', pr_mask[:, :, 1])
        # cv2.waitKey(1)
        # cv2.imshow('shaft', pr_mask[:, :, 2])
        # cv2.waitKey(1)

        RESULT_DIR = self.parameters.result_path
        SAVE_DIR = os.path.join(RESULT_DIR, 'video/check/')

        if self.parameters.save_predicted_images:
            RESULT_DIR = '/home/yuki/Videos/'
            SAVE_DIR = os.path.join(RESULT_DIR, 'prediction_check/')

            if i % 50 == 0:
                cv2.imwrite(SAVE_DIR + str(i + 1) + '-test' + '.png', resized_img)
                # cv2.imwrite(SAVE_DIR + str(i + 1) + '-test_mask1' + '.png', mask_vis_test[:, :, 0] * 255)
                # cv2.imwrite(SAVE_DIR + str(i + 1) + '-test_mask2' + '.png', mask_vis_test[:, :, 1] * 255)
                # cv2.imwrite(SAVE_DIR + str(i + 1) + '-test_mask3' + '.png', mask_vis_test[:, :, 2] * 255)

                cv2.imwrite(SAVE_DIR + str(i + 1) + '-test_predict1' + '.png', pr_mask[:, :, 0])
                cv2.imwrite(SAVE_DIR + str(i + 1) + '-test_predict2' + '.png', pr_mask[:, :, 1])
                # cv2.imwrite(SAVE_DIR + str(i + 1) + '-test_predict3' + '.png', pr_mask[:, :, 2])

        # Unravel indecies of the maximum value in each channel
        tip_instrument = np.array(np.unravel_index(np.argmax(pr_mask[:, :, 0]), pr_mask[:, :, 0].shape))
        tip_shadow_point = np.array(np.unravel_index(np.argmax(pr_mask[:, :, 1]), pr_mask[:, :, 1].shape))
        center = ROIPredictor.get_ROI_center(tip_instrument, tip_shadow_point)

        # Filter out the outliers
        tip_instrument = Detector.outlier_detection(self.previous_point_instrument, tip_instrument, self.parameters.outlier_threshold_instrument, "instrument")
        point_instrument = Detector.outlier_detection(self.previous_point_other, point_instrument, self.parameters.outlier_threshold_another_instrument, "point")  # Filter out the outliers
        
        # If the shadow tip is stuck at the same point, ignore outlier detection
        if len(self.previous_point_shadow) == 0 or np.array_equal(tip_shadow_point, self.previous_point_shadow[-1]) or not self.stuck_counter >= self.parameters.stuck_threshold:
            tip_shadow_point = Detector.outlier_detection(self.previous_point_shadow, tip_shadow_point, self.parameters.outlier_threshold_shadow, "shadow")
            if self.previous_point_shadow and np.array_equal(tip_shadow_point, self.previous_point_shadow[-1]):
                self.stuck_counter += 1
        else:
            self.stuck_counter = 0

        # Add the new points to the deque
        self.previous_point_shadow.append(tip_shadow_point)
        self.previous_point_instrument.append(tip_instrument)
        self.previous_point_other.append(point_instrument)  # Add the new point to the deque

        
        # Ignore outlier shaft point if the angle between the last and current iteration is too large
        shaft_vector_angle = int(self.vector_ignore(point_instrument - tip_instrument))
        if abs(shaft_vector_angle) > int(self.parameters.shaft_vector_angle_threshold):
            print('Exceeding the threshold@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@')
            point_instrument = self.previous_point_other[-1]
        
        # Calculate the EMA of the points (smoothing)
        tip_shadow = self.EMA(self.previous_point_shadow, self.parameters.alpha_shadow)
        point_instrument = self.EMA(self.previous_point_other, self.parameters.alpha_point)

        # Calculate the shaft distance and the tip distance---Section VI and VII of Koyama et al. (2022)
        vec_1 = point_instrument - tip_instrument
        vec_2 = tip_shadow - tip_instrument
        shaft_dis = np.linalg.norm(vec_1.dot(vec_2)/(vec_1.dot(vec_1))*vec_1-vec_2)*ratio
        tip_distance = int(np.linalg.norm(tip_instrument - tip_shadow) * ratio)
            
        # Label the points and display distances in the frame
        cv2.line(resized_img, (int(point_instrument[1]), int(point_instrument[0])), (tip_instrument[1], tip_instrument[0]),
                (255, 0, 0), 4)
        # print('tip_instrument:', tip_instrument)
        # print('point_instrument:', point_instrument)

        cv2.circle(resized_img, (tip_instrument[1], tip_instrument[0]), 5, (255, 0, 0), -1)
        cv2.circle(resized_img, (int(tip_shadow[1]), int(tip_shadow[0])), 5, (0, 0, 255), -1)
        if self.parameters.print_center:
            cv2.circle(resized_img, (int(center[1]), int(center[0])), 10, (0, 255, 0), -1)
    
        cv2.putText(resized_img, 'tip_dis:' + str(tip_distance) + 'px',
                    (int(self.parameters.output_size*250/512), int(self.parameters.output_size*420/512)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        if shaft_dis < 1000:
            cv2.putText(resized_img, 'shaft_dis:' + str(int(shaft_dis)) + 'px',
                        (int(self.parameters.output_size*250/512), int(self.parameters.output_size*450/512)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        cv2.putText(resized_img, 'ROI_size:' + str(int(ROI_dis*2)) + 'px',
                    (int(self.parameters.output_size*250/512), int(self.parameters.output_size*480/512)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        return resized_img


# Make labeled images and masks
def get_image_size(image_path):
    with open(image_path, "r", encoding="utf-8") as f:
        dj = json.load(f)
    w = dj['imageWidth']
    h = dj['imageHeight']
    return w, h

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

def max_distance(point1, point2, center):
    dis1 = np.linalg.norm(center - point1)
    dis2 = np.linalg.norm(center - point2)
    distances = np.array([dis1, dis2])
    return distances.max()

def preprocess_image(image):
    if Parameters.resize_input:
        image = cv2.resize(image, (Parameters.predict_size, Parameters.predict_size))
# # Convert the image to grayscale if it's not already

    # if len(image.shape) == 3:
    #     image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # # Apply bilateral filter for noise reduction
    # image = cv2.GaussianBlur(image, (5, 5), 0)
    image = cv2.bilateralFilter(image, 6, 100, 50)
    # # image = cv2.fastNlMeansDenoisingColored(image, None, 10, 10, 7, 21)

    # # Apply histogram equalization for contrast enhancement
    # image = cv2.equalizeHist(image)

    # # Apply CLAHE (Contrast Limited Adaptive Histogram Equation) to the image
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)                # Convert the image to the LAB color space
    l, a, b = cv2.split(lab)                                    # Split the LAB image into L, A and B channels
    clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(6,6))  # Apply histogram equalization to the L channel
    l = clahe.apply(l)
    lab = cv2.merge((l, a, b))                                  # Merge the equalized L channel back with the A and B channels
    image = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)                # Convert the image back to the BGR color space
    
    # # Min-Max normalization to [0, 1]
    image = (image - np.min(image)) / (np.max(image) - np.min(image))
    # # image = (image - np.mean(image)) / np.std(image)
    # # image = (image - np.mean(image)) / (np.max(image) - np.min(image))
    image = (image * 255).astype(np.uint8)  # Convert the image back to 8-bit

    # # Apply Sobel edge detection
    sobelx = cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=9)
    sobely = cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=9)
    edges = np.hypot(sobelx, sobely)
    edges *= 255.0 / np.max(edges)
    edges = edges.astype(np.uint8)  # Convert the image back to 8-bit
    image = cv2.addWeighted(image, 0.7, edges, 0.3, 0)
    return image

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
    if Parameters.resize_input:
        img = cv2.resize(img, (Parameters.predict_size, Parameters.predict_size))
    cv2.imwrite(path, img)
    return img


# Augmentation
def get_images(input_raw_dir, input_mask_dir, total_image_num):
    n = np.random.choice(total_image_num - 1) + 1
    raw = cv2.imread(input_raw_dir + str(n) + '.png')
    shape = np.shape(raw)
    # raw = cv2.cvtColor(raw, cv2.COLOR_BGR2RGB)
    num_channels = 3 if Parameters.train_other_point else 2
    mask = np.zeros([shape[0], shape[1], num_channels])
    mask[:, :, 0] = cv2.imread(input_mask_dir + str(n) + '-1.png', 0)
    mask[:, :, 1] = cv2.imread(input_mask_dir + str(n) + '-2.png', 0)
    if Parameters.train_other_point:
        mask[:, :, 2] = cv2.imread(input_mask_dir + str(n) + '-3.png', 0)

    return raw, mask

def get_augmentation():
    train_transform = [
        # albu.Blur(), # blur_limit=21, p=1
        # albu.MotionBlur(), # blur_limit=21
        # albu.GaussianBlur(), # blur_limit=21
        # albu.GlassBlur(),
        # albu.ISONoise(),
        # albu.MultiplicativeNoise(),
        # albu.Downscale(),
        # albu.ColorJitter(),
        # albu.RandomGamma(),
        # albu.RandomBrightnessContrast(), #####
        # albu.GaussNoise(p=0.2),
        # albu.ImageCompression(),
        albu.Rotate(limit=20),
        # albu.RandomResizedCrop(1024, 1024, scale=(0.8, 1.0)),
        # albu.OpticalDistortion(),
        # albu.GridDistortion(),
        # albu.ElasticTransform(),
        # albu.RandomGridShuffle(),
        # albu.HueSaturationValue(),
        # albu.RGBShift(),
        # albu.ChannelDropout(),
        # albu.Normalize(),
        # albu.RandomBrightness(),
        # albu.RandomContrast(),
        albu.Compose([
            albu.OneOf([
                albu.Blur(), 
                albu.MotionBlur(), 
                albu.GaussianBlur(), 
                albu.GlassBlur(),
            ], p=0.5),
            albu.OneOf([
                albu.ISONoise(),
                albu.MultiplicativeNoise(),
                albu.Downscale(),
            ], p=0.5),
            albu.OneOf([
                albu.ColorJitter(),
                albu.RandomGamma(),
                albu.RandomBrightnessContrast(brightness_limit=(-0.05, 0.2), contrast_limit=(-0.2, 0.2)),
            ], p=0.5),
        ]),
    ]
    return albu.OneOf(train_transform)

def save_augmented_images(save_raw_dir, save_mask_dir, raw_image, mask_image, iteration, first_num):
    raw_output_path = save_raw_dir + str(iteration + first_num) + '.png'
    cv2.imwrite(raw_output_path, raw_image)

    mask_output_path1 = save_mask_dir + str(iteration + first_num) + '-1.png'
    cv2.imwrite(mask_output_path1, mask_image[:, :, 0])
    mask_output_path2 = save_mask_dir + str(iteration + first_num) + '-2.png'
    cv2.imwrite(mask_output_path2, mask_image[:, :, 1])
    if Parameters.train_other_point:
        mask_output_path3 = save_mask_dir + str(iteration + first_num) + '-3.png'
        cv2.imwrite(mask_output_path3, mask_image[:, :, 2])

def copy_files(get_raw_dir, get_mask_dir, output_raw_dir, output_mask_dir, input_iteration, output_iteration):
    shutil.copyfile(get_raw_dir + str(input_iteration) + '.png', output_raw_dir + str(output_iteration) + '.png')
    shutil.copyfile(get_mask_dir + str(input_iteration) + '-1.png', output_mask_dir + str(output_iteration) + '-1.png')
    shutil.copyfile(get_mask_dir + str(input_iteration) + '-2.png', output_mask_dir + str(output_iteration) + '-2.png')
    if Parameters.train_other_point:
        shutil.copyfile(get_mask_dir + str(input_iteration) + '-3.png', output_mask_dir + str(output_iteration) + '-3.png')


# Video output
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
