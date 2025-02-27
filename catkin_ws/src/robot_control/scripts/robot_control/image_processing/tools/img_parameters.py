import numpy as np


class ImageParameters:
    """
    This class contains parameters related to the 4K image processing for the forceps tip, shaft, and shadow detection. 
    There are two different models used for the tip and ROI detection. Parameters in this class are for the predict_node, video_node,
    image_show, get_positioning_points, and detection optimization. 
    """
    # Common parameters
    original_w = 3840  # original image size from the camera
    original_h = 2160
    best_model = '/home/yuki/Documents/MyProjects/4k_tip_prediction/result/for_eye_model2/4K_512_ver3/best_model.pth'  # tip detection model
    ROI_model = '/home/yuki/Documents/MyProjects/4k_tip_prediction/result/ROI_eye_model1/4K_270_ver2/best_model.pth'  # ROI detection model

    # Parameters for keypoint_predict_node (@@--- ALIGN WITH C++ SIDE ---@@)
    output_size = 512   # size of the output ROI image
    predict_size = 256  # downscaled for faster processing
    
    # Parameters for ROI_predict_node
    ROI_resize_size = 270  # height size of the resized frame for ROI center detection
    margin = 240
    threshold1 = output_size  # ROI size scales with distance between tip and shadow
    threshold2 = threshold1 * 1.5
    threshold3 = threshold1 * 2

    print_debugging_information = False
    print_outlier = False
    print_predict_node_time = False
    print_time_from_get_to_predict = False
    save_predicted_images = False

    # Parameters for video_node
    video_dir = '/home/Documents/MyProjects/4K_tip_prediction/result/for_eye_model2/4K_256_ver4/4K/'
    print_video_node_time = False

    # Parameters for image_show node
    fontsize = 0.5
    video_save_fps = 60
    print_image_show_time = False
    print_time_from_get_to_show = False
    print_center = False

    # Parameters for get_positioning_points node
    hsvLower1 = np.array([0, 64, 70])      # Lower limit
    hsvUpper1 = np.array([30, 255, 255])   # Upper limit
    hsvLower2 = np.array([150, 64, 70])    # Lower limit
    hsvUpper2 = np.array([179, 255, 255])  # Upper limit
    erode_iteration = 1
    dilate_iteration = 2
    min_point_area = 50
    num_of_points = 5
    update_pace = 100
    image_debug = False

    # Detection Optimization parameters
    mixed_precision = False
    SMA_smoothing_N = 6
    outlier_threshold_roi = 200
    freeze_threshold = 10
    alpha_shadow = 0.6
    alpha_point = 0.6
    EMA_shadow_N = 8
    EMA_point_N = 5
    outlier_threshold_instrument = 175
    outlier_threshold_shadow = 175
    outlier_threshold_another_instrument = 10800
    shaft_vector_angle_threshold = 110
    stuck_threshold = 20

