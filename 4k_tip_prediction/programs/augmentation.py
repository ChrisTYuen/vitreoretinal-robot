import os
import random
import cv2
from tools import Parameters
from tools import functions as fn


def augmentation():
    """
    This function performs data augmentation on images and their corresponding masks. It calculates the number of images needed for 
    augmentation, applies transformations to generate augmented images, and saves them. It also splits the original and augmented 
    images into training, testing, and validation sets.
    """
    parameters = Parameters.Parameters()

    image_num_after_augmentation = parameters.image_num_after_augmentation

    original_num = len(os.listdir(parameters.raw_resized_path))
    original_num_for_train = parameters.original_num_for_train
    original_num_for_test = int((original_num - original_num_for_train) / 2)
    original_num_for_val = original_num_for_test

    augmentation_num = image_num_after_augmentation - original_num
    aug_num_for_train = parameters.aug_num_for_train
    aug_num_for_test = int((augmentation_num - aug_num_for_train) / 2)
    aug_num_for_val = aug_num_for_test

    transforms = fn.get_augmentation()

    if not os.path.exists(parameters.raw_augmented_path):
        os.mkdir(parameters.raw_augmented_path)

    if not os.path.exists(parameters.mask_augmented_path):
        os.mkdir(parameters.mask_augmented_path)

    k = 0
    while k < augmentation_num:
        k = k + 1
        print('augmenting...: ' + str(k) + '/' + str(augmentation_num))
        raw_original, mask_original = fn.get_images(parameters.raw_resized_path, parameters.mask_resized_path, original_num)
        augmented = transforms(image=raw_original, mask=mask_original)
        raw_augmented, mask_augmented = augmented['image'], augmented['mask']
        fn.save_augmented_images(parameters.raw_augmented_path, parameters.mask_augmented_path, raw_augmented, mask_original, k,
                                 original_num)
        del raw_augmented, mask_augmented, augmented, raw_original, mask_original

    raw_list = list(range(original_num))
    random.shuffle(raw_list)

    aug_list = list(range(augmentation_num))
    random.shuffle(aug_list)

    if not os.path.exists(parameters.dataset_train_path):
        os.mkdir(parameters.dataset_train_path)

    if not os.path.exists(parameters.dataset_test_path):
        os.mkdir(parameters.dataset_test_path)

    if not os.path.exists(parameters.dataset_val_path):
        os.mkdir(parameters.dataset_val_path)

    if not os.path.exists(parameters.dataset_train_mask_path):
        os.mkdir(parameters.dataset_train_mask_path)

    if not os.path.exists(parameters.dataset_test_mask_path):
        os.mkdir(parameters.dataset_test_mask_path)

    if not os.path.exists(parameters.dataset_val_mask_path):
        os.mkdir(parameters.dataset_val_mask_path)

    i = 0
    while i < original_num_for_train:
        fn.copy_files(parameters.raw_resized_path, parameters.mask_resized_path,
                      parameters.dataset_train_path, parameters.dataset_train_mask_path, raw_list[i] + 1, i + 1)
        i = i + 1
    print(str(original_num_for_train) + ' original images copied to train and train_mask folder!')

    while i < original_num_for_train + original_num_for_test:
        fn.copy_files(parameters.raw_resized_path, parameters.mask_resized_path,
                      parameters.dataset_test_path, parameters.dataset_test_mask_path, raw_list[i] + 1,
                      i - original_num_for_train + 1)
        i = i + 1
    print(str(original_num_for_test) + ' original images copied to test and test_mask folder!')

    while i < original_num_for_train + original_num_for_test + original_num_for_val:
        fn.copy_files(parameters.raw_resized_path, parameters.mask_resized_path,
                      parameters.dataset_val_path, parameters.dataset_val_mask_path, raw_list[i] + 1,
                      i - original_num_for_train - original_num_for_test + 1)
        i = i + 1
    print(str(original_num_for_val) + ' original images copied to val and val_mask folder!')

    i = 0
    while i < aug_num_for_train:
        fn.copy_files(parameters.raw_augmented_path, parameters.mask_augmented_path,
                      parameters.dataset_train_path, parameters.dataset_train_mask_path,
                      aug_list[i] + original_num + 1, i + original_num_for_train + 1)
        i = i + 1
    print(str(aug_num_for_train) + ' augmented images copied to train and train_mask folder!')

    while i < aug_num_for_train + aug_num_for_test:
        fn.copy_files(parameters.raw_augmented_path, parameters.mask_augmented_path,
                      parameters.dataset_test_path, parameters.dataset_test_mask_path,
                      aug_list[i] + original_num + 1,
                      i - aug_num_for_train + original_num_for_test + 1)
        i = i + 1
    print(str(aug_num_for_test) + ' augmented images copied to test and test_mask folder!')

    while i < aug_num_for_train + aug_num_for_test + aug_num_for_val:
        fn.copy_files(parameters.raw_augmented_path, parameters.mask_augmented_path,
                      parameters.dataset_val_path, parameters.dataset_val_mask_path,
                      aug_list[i] + original_num + 1,
                      i - aug_num_for_train - aug_num_for_test + original_num_for_val + 1)
        i = i + 1
    print(str(aug_num_for_val) + ' augmented images copied to val and val_mask folder!')


if __name__ == '__main__':
    augmentation()
