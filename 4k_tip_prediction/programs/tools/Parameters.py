import numpy as np
class Parameters:
    ####### Path #######
    model = 'for_eye_model1'
    image_quality = '4K_256_ver1'
    original_video_quality = '4K'
    folder_path = '/home/yuki/Documents/MyProjects/4k_tip_prediction/'

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

    ####### 0_extract_frames #######
    frame_interval = 45

    ####### 1_rename #######
    start_num = 600
    total_image = 70

    ####### 2_make_labeled_image #######
    debug = True
    binary = False
    threshold = 100
    resize_size = 256  #256  #512
    # resize_size_w = 256
    margin =70   #100  #100
    mask_specific_image = False
    specific_image_num = 239
    mask_sigma =400  #800  #1500
    slide =40  #40  #80
    threshold1 = resize_size
    threshold2 = resize_size*2  #*2  #* 3/2
    threshold3 = resize_size*3  #*3  #* 2

    ####### 3_augmentation #######
    # 70% training, 15% validation, 15% testing
    image_num_after_augmentation = 3000
    original_num_for_train = 400
    aug_num_for_train = 1680

    ####### 4_train_confidence_map #######
    CLASSES = ['tip_instrument', 'tip_shadow', 'tip_another_instrument']
    ENCODER = 'resnet34'  # 'resnet18'
    ENCODER_WEIGHTS = 'imagenet'
    ACTIVATION = 'sigmoid'
    DEVICE = 'cuda'
    epoch = 20
    train_batch_size = 5
    resize_size_list = [resize_size, resize_size]

    ####### 5_video_output_confidence_map #######
    predict_size = resize_size
    print_center = False
    print_time = False
