import numpy as np


class ImageParameters:
    """
    This class contains parameters related to image processing.
    """
    # common
    original_w = 3840
    original_h = 2160
    best_model = '/home/yuki/Documents/eye_surgery/image_processing/4K_tip_prediction/result/eye_model3/4K_256_ver3/best_model.pth'

    # parameters for predict_node
    predict_size = 256
    output_size = 256
    threshold1 = predict_size
    threshold2 = threshold1*2
    threshold3 = threshold1*3
    margin = 70
    print_img_size = False
    print_predict_node_time = False
    print_time_from_get_to_predict = False
    save_predicted_images = False
    ROI_center_init = np.array([original_w/2+100, original_h/2+500])

    # parameters for video_node
    video_dir = '/home/yuki/Documents/eye_surgery/image_processing/4K_tip_prediction/original_video/eye_model3/4K/'
    print_video_node_time = False

    # parameters for image_show node
    fontsize = 0.5
    video_save_fps = 60
    print_image_show_time = False
    print_time_from_get_to_show = False
    print_center = False

    # parameters for get_positioning_points node
    hsvLower1 = np.array([0, 64, 70])  # Lower limit
    hsvUpper1 = np.array([30, 255, 255])  # Upper limit
    hsvLower2 = np.array([150, 64, 70])  # Lower limit
    hsvUpper2 = np.array([179, 255, 255])  # Upper limit
    erode_iteration = 1
    dilate_iteration = 2
    min_point_area = 50
    num_of_points = 5
    update_pace = 100
    image_debug = False

