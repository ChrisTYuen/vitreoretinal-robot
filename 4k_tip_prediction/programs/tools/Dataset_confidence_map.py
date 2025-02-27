import os
import cv2
import numpy as np
from torch.utils.data import Dataset as BaseDataset

import sys
sys.path.append('/home/yuki/Documents/MyProjects/4k_tip_prediction/programs/tools')
from Parameters import Parameters


class Dataset(BaseDataset):
    if Parameters.train_other_point:
        CLASSES = ['tip_instrument', 'tip_shadow', 'tip_another_instrument']
    else:
        CLASSES = ['tip_instrument', 'tip_shadow']

    def __init__(self, images_dir, masks_dir, image_size, classes=None, augmentation=None, preprocessing=None):
        self.ids = os.listdir(images_dir)
        self.data_num = len(self.ids)
        self.images_dir = images_dir
        self.image_size = image_size
        self.masks_dir = masks_dir

        self.augmentation = augmentation
        self.preprocessing = preprocessing

    def __getitem__(self, i):
        image = cv2.imread(self.images_dir+'/'+str(i+1)+'.png')
        hr = int(self.image_size[1])
        wr = int(self.image_size[0])
        image = cv2.resize(image, (wr, hr))
        # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        mask1 = cv2.imread(self.masks_dir + '/' + str(i+1) + '-1.png', 0)
        mask2 = cv2.imread(self.masks_dir + '/' + str(i+1) + '-2.png', 0)
        if Parameters.train_other_point:
            mask3 = cv2.imread(self.masks_dir + '/' + str(i+1) + '-3.png', 0)

        # extract certain classes from mask (e.g. cars)
        num_channels = len(self.CLASSES)
        masks = np.zeros([np.shape(mask1)[0], np.shape(mask1)[1], num_channels])
        masks[:, :, 0] = mask1/255
        masks[:, :, 1] = mask2/255
        if Parameters.train_other_point:
            masks[:, :, 2] = mask3/255
        mask = cv2.resize(masks, (wr, hr))

        # apply augmentations
        if self.augmentation:
            sample = self.augmentation(image=image, mask=mask)
            image, mask = sample['image'], sample['mask']

        # apply preprocessing
        if self.preprocessing:
            sample = self.preprocessing(image=image, mask=mask)
            image, mask = sample['image'], sample['mask']

        return image, mask

    def __len__(self):
        return len(self.ids)

