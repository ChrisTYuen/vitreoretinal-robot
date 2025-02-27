import numpy as np
class Parameters:
    """
    This file contains the parameters used in the tip detection pipeline for developing the keypoint detection model.
    There are two main modes: ROI_training and train_other_point. ROI_training is used to train the ROI detection model,
    while without is used to train the keypoint detection model with finding the tip of the instrument and its shadow.
    The point on the shaft is currently trained with the ROI model as its more accurate by observing the full microscope image.
    """

    # Important parameters
    ROI_training = False  # If false, train keypoint detection model
    train_other_point = True  # If true, train keypoint detection model with additional point

    ####### Path #######
    # Ensure model name and version is correct
    model = 'for_eye_model2'
    ROI_model = 'ROI_eye_model1'
    image_quality = '4K_512_ver3'
    ROI_image_quality = '4K_270_ver2'
    original_video_quality = '4K'
    folder_path = '/home/yuki/Documents/MyProjects/4k_tip_prediction/'

    if ROI_training:
        video_path = folder_path + 'original_video/' + ROI_model + '/' + original_video_quality + '/'
        extracted_frames_path = folder_path + 'workspace/extracted_frames/'
        json_and_raw_path = folder_path + 'workspace/' + ROI_model + '/' + ROI_image_quality + '/json_and_raw/'
        debug_path = folder_path + 'workspace/' + ROI_model + '/' + ROI_image_quality + '/debug/'
        raw_resized_path = folder_path + 'workspace/' + ROI_model + '/' + ROI_image_quality + '/raw_resized/'
        mask_resized_path = folder_path + 'workspace/' + ROI_model + '/' + ROI_image_quality + '/mask_resized/'
        raw_augmented_path = folder_path + 'workspace/' + ROI_model + '/' + ROI_image_quality + '/raw_augmented/'
        mask_augmented_path = folder_path + 'workspace/' + ROI_model + '/' + ROI_image_quality + '/mask_augmented/'

        dataset_path = folder_path + 'dataset/' + ROI_model + '/' + ROI_image_quality + '/'
        dataset_train_path = folder_path + 'dataset/' + ROI_model + '/' + ROI_image_quality + '/train/'
        dataset_test_path = folder_path + 'dataset/' + ROI_model + '/' + ROI_image_quality + '/test/'
        dataset_val_path = folder_path + 'dataset/' + ROI_model + '/' + ROI_image_quality + '/val/'
        dataset_train_mask_path = folder_path + 'dataset/' + ROI_model + '/' + ROI_image_quality + '/train_mask/'
        dataset_test_mask_path = folder_path + 'dataset/' + ROI_model + '/' + ROI_image_quality + '/test_mask/'
        dataset_val_mask_path = folder_path + 'dataset/' + ROI_model + '/' + ROI_image_quality + '/val_mask/'

        result_path = folder_path + 'result/' + ROI_model + '/' + ROI_image_quality + '/'

    else:
        video_path = folder_path + 'original_video/' + model + '/' + original_video_quality + '/'
        extracted_frames_path = folder_path + 'workspace/extracted_frames/'
        json_and_raw_path = folder_path + 'workspace/' + model + '/' + image_quality + '/json_and_raw/'
        debug_path = folder_path + 'workspace/' + model + '/' + image_quality + '/debug/'
        raw_resized_path = folder_path + 'workspace/' + model + '/' + image_quality + '/raw_resized/'
        mask_resized_path = folder_path + 'workspace/' + model + '/' + image_quality + '/mask_resized/'
        raw_augmented_path = folder_path + 'workspace/' + model + '/' + image_quality + '/raw_augmented/'
        mask_augmented_path = folder_path + 'workspace/' + model + '/' + image_quality + '/mask_augmented/'

        dataset_path = folder_path + 'dataset/' + model + '/' + image_quality + '/'
        dataset_train_path = folder_path + 'dataset/' + model + '/' + image_quality + '/train/'
        dataset_test_path = folder_path + 'dataset/' + model + '/' + image_quality + '/test/'
        dataset_val_path = folder_path + 'dataset/' + model + '/' + image_quality + '/val/'
        dataset_train_mask_path = folder_path + 'dataset/' + model + '/' + image_quality + '/train_mask/'
        dataset_test_mask_path = folder_path + 'dataset/' + model + '/' + image_quality + '/test_mask/'
        dataset_val_mask_path = folder_path + 'dataset/' + model + '/' + image_quality + '/val_mask/'

        result_path = folder_path + 'result/' + model + '/' + image_quality + '/'
    
    ROI_path = folder_path + 'result/' + ROI_model + '/' + ROI_image_quality + '/'


    ####### 0_extract_frames #######
    frame_interval = 40

    ####### 1_rename #######
    start_num = 577
    total_image = 572

    ####### 2_make_labeled_image #######
    if ROI_training:
        image_preprocessing = False 
        resize_input = False
    else:
        image_preprocessing = True
        resize_input = True  # resize the input image/mask for lighter NN
        
    ROI_resize_size = 270  #360  # height of resized frame for ROI center detection 
    output_size = 512  #256  #512  # size of the output image for display
    predict_size = 256  # input size for NN prediction model
    threshold1 = output_size
    threshold2 = threshold1 * 1.5 #*2  #* 1.5
    threshold3 = threshold2 * 2 #*3  #* 2
    threshold = 100
    margin = 240   #100 #150 #200
    mask_sigma = 400  #800  #1500
    slide = 40  #40  #80

    mask_specific_image = False
    specific_image_num = 63
    debug = False
    binary = False

    ####### 3_augmentation #######
    # 70% training, 15% validation, 15% testing
    image_num_after_augmentation = 6000
    original_num_for_train = 700

    ####### 4_train_confidence_map #######
    # train_other_point = True
    if train_other_point:
        CLASSES = ['tip_instrument', 'tip_shadow', 'tip_another_instrument']
        ACTIVATION = 'sigmoid'
    else:
        CLASSES = ['tip_instrument', 'tip_shadow']
        ACTIVATION = 'softmax'
    ENCODER = 'resnet18'  # 'resnet50' 
    ENCODER_WEIGHTS = 'imagenet'
    DEVICE = 'cuda'
    epoch = 80
    lr_change_epoch1 = 30
    lr_change_epoch2 = 60
    lr_initial = 1e-3
    lr1 = 1e-4
    lr2 = 1e-5
    weight_decay = 1e-5
    train_batch_size = 32

    ####### 5_video_output_confidence_map #######
    save_predicted_images = False
    print_center = False
    print_time = False
    SMA_smoothing_N = 6
    outlier_threshold_roi = 200
    freeze_threshold = 10
    alpha_shadow = 0.75
    alpha_point = 0.78
    EMA_shadow_N = 15
    EMA_point_N = 5
    outlier_threshold_instrument = 100
    outlier_threshold_shadow = 100
    outlier_threshold_another_instrument = 200
    shaft_vector_angle_threshold = 6
    stuck_threshold = 20
