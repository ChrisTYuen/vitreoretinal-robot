import glob
import cv2
import os
import numpy as np
from tools import functions as fn
from tools import Parameters


def make_labeled_image():
    """
    This function calculates the center and maximum distance between the points specified in the JSON files, and uses this information to crop the image 
    around the region of interest. Depending on the maximum distance, the image is resized to a certain size. If the image is binary, it is thresholded. 
    The function also creates mask images for each point of interest using a Gaussian confidence map, which are saved in a specified directory. If in debug 
    mode, it generates additional outputs for debugging purposes, such as marking the points of interest and the cropping area on the original image.
    """
    parameters = Parameters.Parameters()
    threshold1 = parameters.threshold1
    threshold2 = parameters.threshold2
    threshold3 = parameters.threshold3

    # Check if all image and json files are present
    files = os.listdir(parameters.json_and_raw_path)
    for k in range(len(files)//2):
        image_path = parameters.json_and_raw_path + str(k + 1) + '.png'
        json_path = parameters.json_and_raw_path + str(k + 1) + '.json'
        if not (os.path.isfile(image_path) or os.path.isfile(json_path)):
            print(f"Image or JSON file not found: {image_path}")
            return

    if parameters.mask_specific_image:
        json_num = parameters.specific_image_num
    else:
        json_num = len(glob.glob1(parameters.json_and_raw_path, "*.json"))

    if parameters.mask_specific_image:
        start_num = parameters.specific_image_num - 1
    else:
        start_num = 0

    if parameters.debug:
        if not os.path.exists(parameters.debug_path):
            os.mkdir(parameters.debug_path)

    if not os.path.exists(parameters.raw_resized_path):
        os.mkdir(parameters.raw_resized_path)

    if not os.path.exists(parameters.mask_resized_path):
        os.mkdir(parameters.mask_resized_path)

    for k in range(start_num, json_num):
        if parameters.binary:
            image = cv2.imread(parameters.json_and_raw_path + str(k + 1) + '.png', 0)
            image_original = cv2.imread(parameters.json_and_raw_path + str(k + 1) + '.png')
        else:
            image = cv2.imread(parameters.json_and_raw_path + str(k + 1) + '.png')
        input_json_path = parameters.json_and_raw_path + str(k + 1) + '.json'
        w, h = fn.get_image_size(input_json_path)

        x1, y1 = fn.get_instrument_tip_from_json(input_json_path)
        x2, y2 = fn.get_shadow_tip_from_json(input_json_path)
        x3, y3 = fn.get_another_instrument_point_from_json(input_json_path)

        tip_instrument = np.array([x1, y1])
        tip_shadow = np.array([x2, y2])
        point_instrument = np.array([x3, y3])

        center = fn.get_ROI_center(tip_instrument, point_instrument, tip_shadow)

        max_dis = fn.max_distance(tip_instrument, tip_shadow, point_instrument, center)

        if parameters.debug:
            debug_image = cv2.imread(parameters.json_and_raw_path + str(k + 1) + '.png')
            cv2.circle(debug_image, (int(x1), int(y1)), 10, (255, 0, 0), -1)
            cv2.circle(debug_image, (int(x2), int(y2)), 10, (0, 255, 0), -1)
            cv2.circle(debug_image, (int(x3), int(y3)), 10, (0, 0, 255), -1)
            cv2.circle(debug_image, (int(center[0]), int(center[1])), 10, (0, 255, 255), -1)

        if max_dis + parameters.margin <= threshold1 / 2:
            dis = threshold1 / 2
            print("Image "+ str(k+1) + "/" + str(json_num) + " not_resized")
        elif max_dis + parameters.margin <= threshold2 / 2:
            dis = threshold2 / 2
            print("Image "+ str(k+1) + "/" + str(json_num) + " resized: " + str(int(threshold2)) + "×" + str(int(threshold2)) + " -> " + str(
                parameters.resize_size) + "×" + str(parameters.resize_size))
        elif max_dis + parameters.margin <= threshold3 / 2:
            dis = threshold3 / 2
            print("Image "+ str(k+1) + "/" + str(json_num) + " resized: " + str(int(threshold3)) + "×" + str(int(threshold3)) + " -> " + str(
                parameters.resize_size) + "×" + str(parameters.resize_size))
        else:
            dis = h / 2
            print("Image "+ str(k+1) + "/" + str(json_num) + " resized: " + str(int(h)) + "×" + str(int(h)) + " -> " + str(parameters.resize_size) + "×" + str(
                parameters.resize_size))

        if center[0] - dis < 0:
            center[0] = dis
        if center[1] - dis < 0:
            center[1] = dis
        if center[0] + dis >= w:
            center[0] = w - dis
        if center[1] + dis >= h:
            center[1] = h - dis

        slide = parameters.slide
        if y3 - center[1]+slide*(dis/threshold1*2) > dis:
            y3_old = y3
            y3 = center[1]+dis-slide*(dis/threshold1*2)
            x3 = x1-(x1-x3)/(y1-y3_old)*(y1-y3)
        if x3 - center[0]+slide*(dis/threshold1*2) > dis:
            x3_old = x3
            x3 = center[0]+dis-slide*(dis/threshold1*2)
            y3 = y1-(y1-y3)/(x1-x3_old)*(x1-x3)

        center = center.astype('uint64')
        left_top = center - np.array([dis, dis])
        left_top = left_top.astype('uint64')
        right_bottom = left_top + np.array([2 * dis, 2 * dis])
        right_bottom = right_bottom.astype('uint64')

        if parameters.debug:
            cv2.circle(debug_image, left_top, 10, (255, 255, 0), -1)
            cv2.circle(debug_image, right_bottom, 10, (255, 0, 255), -1)
            cv2.circle(debug_image, (int(x3), int(y3)), 10, (255, 255, 255), -1)
            cv2.circle(image, (int(x1), int(y1)), 10, (255, 0, 0), -1)
            cv2.circle(image, (int(x2), int(y2)), 10, (0, 255, 0), -1)
            cv2.circle(image, (int(x3), int(y3)), 10, (0, 0, 255), -1)
            cv2.circle(image, (int(center[0]), int(center[1])), 10, (0, 255, 255), -1)

        if image is None:
            print(f"Could not open image at {parameters.json_and_raw_path}. Please check the image file.")
            return
        else:
            image = image[left_top[1]:right_bottom[1], left_top[0]:right_bottom[0]]
        if parameters.binary:
            image_original = image_original[left_top[1]:right_bottom[1], left_top[0]:right_bottom[0]]

        if parameters.binary:
            ret, image = cv2.threshold(image, parameters.threshold, 255, cv2.THRESH_BINARY)

        output_image_instrument_path = parameters.mask_resized_path + str(k + 1) + '-1.png'
        fn.make_mask_image_confidence_map(parameters.resize_size, parameters.resize_size,
                                          (x1 - left_top[0])/(right_bottom[0] - left_top[0])*parameters.resize_size,
                                          (y1 - left_top[1])/(right_bottom[1] - left_top[1])*parameters.resize_size,
                                          parameters.mask_sigma, output_image_instrument_path)
        output_image_shadow_path = parameters.mask_resized_path + str(k + 1) + '-2.png'
        fn.make_mask_image_confidence_map(parameters.resize_size, parameters.resize_size,
                                          (x2 - left_top[0])/(right_bottom[0] - left_top[0])*parameters.resize_size,
                                          (y2 - left_top[1])/(right_bottom[1] - left_top[1])*parameters.resize_size,
                                          parameters.mask_sigma, output_image_shadow_path)
        output_image_instrument_another_path = parameters.mask_resized_path + str(k + 1) + '-3.png'
        fn.make_mask_image_confidence_map(parameters.resize_size, parameters.resize_size,
                                          (x3 - left_top[0])/(right_bottom[0] - left_top[0])*parameters.resize_size,
                                          (y3 - left_top[1])/(right_bottom[1] - left_top[1])*parameters.resize_size,
                                          parameters.mask_sigma, output_image_instrument_another_path)

        if not max_dis + parameters.margin <= threshold1 / 2:
            image = cv2.resize(image, (parameters.resize_size, parameters.resize_size))

        cv2.imwrite(parameters.raw_resized_path + str(k + 1) + '.png', image)
        if parameters.binary:
            cv2.imwrite(parameters.raw_resized_path + str(k + 1) + '-original.png', image_original)


        if parameters.debug:
            output_debug_image_path = parameters.debug_path + str(k + 1) + '-debug' + '.png'
            cv2.imwrite(output_debug_image_path, debug_image)
    
    print("Image resizing and labeling complete.")


if __name__ == '__main__':
    make_labeled_image()
