import os
from tools.Dataset_confidence_map import Dataset
from tools import functions
import torch
from torch.utils.data import DataLoader
import segmentation_models_pytorch as smp
import numpy as np
import cv2
import albumentations as albu
from tools import Parameters


def test_confidence_map():
    parameters = Parameters.Parameters()

    image_size = [parameters.predict_size, parameters.predict_size]
    CLASSES = parameters.CLASSES
    ENCODER = parameters.ENCODER
    ENCODER_WEIGHTS = parameters.ENCODER_WEIGHTS
    DEVICE = parameters.DEVICE

    DATA_DIR = parameters.dataset_path
    RESULT_DIR = parameters.result_path
    x_test_dir = os.path.join(DATA_DIR, 'test')
    y_test_dir = os.path.join(DATA_DIR, 'test_mask')

    os.environ['CUDA_VISIBLE_DEVICES'] = '0'
    if not torch.cuda.is_available():
        raise Exception("GPU not available. CPU training will be too slow.")
    print("device name", torch.cuda.get_device_name(0))

    best_model = torch.load(RESULT_DIR + 'best_model.pth')

    SAVE_DIR = os.path.join(RESULT_DIR, 'test_image/')

    preprocessing_fn = smp.encoders.get_preprocessing_fn(ENCODER, ENCODER_WEIGHTS)

    test_dataset = Dataset(x_test_dir, y_test_dir, image_size,
                           preprocessing=functions.get_preprocessing(preprocessing_fn), classes=CLASSES)

    test_dataloader = DataLoader(test_dataset)

    loss = smp.utils.losses.MSELoss()
    metrics = [smp.utils.metrics.IoU(threshold=0.5)]

    # evaluate model on test set
    test_epoch = smp.utils.train.ValidEpoch(model=best_model, loss=loss, metrics=metrics, device=DEVICE)
    logs = test_epoch.run(test_dataloader)

    # test dataset without transformations for image visualization
    test_dataset_vis = Dataset(x_test_dir, y_test_dir, image_size, classes=CLASSES)

    if not os.path.exists(RESULT_DIR + 'test_image'):
        os.mkdir(RESULT_DIR + 'test_image')

    for i in range(25):
        n_test = np.random.choice(len(test_dataset))
        print(n_test)
        print(test_dataset_vis.ids[n_test])
        print(test_dataset.ids[n_test])

        image_test, mask_test = test_dataset[n_test]
        image_vis_test = test_dataset_vis[n_test][0].astype('uint8')
        mask_vis_test = test_dataset_vis[n_test][1]

        cv2.imwrite(SAVE_DIR + str(i + 1) + '-test' + '.png', image_vis_test)
        cv2.imwrite(SAVE_DIR + str(i + 1) + '-test_mask1' + '.png', mask_vis_test[:, :, 0] * 255)
        cv2.imwrite(SAVE_DIR + str(i + 1) + '-test_mask2' + '.png', mask_vis_test[:, :, 1] * 255)
        cv2.imwrite(SAVE_DIR + str(i + 1) + '-test_mask3' + '.png', mask_vis_test[:, :, 2] * 255)

        x_tensor = torch.from_numpy(image_test).to(DEVICE).unsqueeze(0)
        pr_mask_train = best_model.predict(x_tensor)
        pr_mask_train = (pr_mask_train.squeeze().cpu().numpy())
        pr_mask_train = pr_mask_train * 255
        pr_mask_train = np.transpose(pr_mask_train, (1, 2, 0))
        cv2.imwrite(SAVE_DIR + str(i + 1) + '-test_predict1' + '.png', pr_mask_train[:, :, 0])
        cv2.imwrite(SAVE_DIR + str(i + 1) + '-test_predict2' + '.png', pr_mask_train[:, :, 1])
        cv2.imwrite(SAVE_DIR + str(i + 1) + '-test_predict3' + '.png', pr_mask_train[:, :, 2])


def to_tensor(x, **kwargs):
    """
    to_tensor(x, **kwargs) transposes input images for prediction.
    """
    return x.transpose(2, 0, 1).astype('float32')


def get_preprocessing(preprocessing_fn):
    """
    get_preprocessing(preprocessing_fn) defines preprocessing for input images.
    """
    _transform = [
        albu.Lambda(image=preprocessing_fn),
        albu.Lambda(image=to_tensor, mask=to_tensor),
    ]
    return albu.Compose(_transform)


def test_confidence_map2():
    parameters = Parameters.Parameters()
    best_model_path = '/home/yuki/Documents/eye_surgery/image_processing/4K_tip_prediction/result/eye_model3/4K_256_ver2/best_model.pth'
    best_model = torch.load(best_model_path)

    ENCODER = 'resnet18'
    ENCODER_WEIGHTS = 'imagenet'
    DEVICE = 'cuda'

    preprocessing_fn = smp.encoders.get_preprocessing_fn(ENCODER, ENCODER_WEIGHTS)
    preprocessing = get_preprocessing(preprocessing_fn)

    DATA_DIR = parameters.dataset_path
    test_dir = os.path.join(DATA_DIR, 'test')
    test_mask_dir = os.path.join(DATA_DIR, 'test_mask')
    test_list = os.listdir(test_dir)

    RESULT_DIR = parameters.result_path
    SAVE_DIR = os.path.join(RESULT_DIR, 'test_image/')

    if not os.path.exists(RESULT_DIR + 'test_image'):
        os.mkdir(RESULT_DIR + 'test_image')

    for i in range(25):
        n_test = np.random.choice(len(test_list))
        print(test_list[n_test])

        test_img_path = test_dir + "/" + test_list[n_test]
        test_mask_path = test_mask_dir + "/" + test_list[n_test]
        test_img = cv2.imread(test_img_path)
        # test_mask1 = cv2.imread(test_mask_path + "-1.")
        # test_mask2 = cv2.imread(test_mask_path)
        # test_mask3 = cv2.imread(test_mask_path)

        x_img = preprocessing(image=test_img)['image']
        x_tensor = torch.from_numpy(x_img).to(DEVICE).unsqueeze(0)
        pr_mask = best_model.predict(x_tensor)  # torch.tensor
        pr_mask = np.transpose((pr_mask.squeeze().cpu().numpy()) * 255, (1, 2, 0))

        print(test_dir + "/" + test_list[n_test])

        cv2.imwrite(SAVE_DIR + str(i + 1) + '-test' + '.png', test_img)
        # cv2.imwrite(SAVE_DIR + str(i + 1) + '-test_mask1' + '.png', mask_vis_test[:, :, 0] * 255)
        # cv2.imwrite(SAVE_DIR + str(i + 1) + '-test_mask2' + '.png', mask_vis_test[:, :, 1] * 255)
        # cv2.imwrite(SAVE_DIR + str(i + 1) + '-test_mask3' + '.png', mask_vis_test[:, :, 2] * 255)

        cv2.imwrite(SAVE_DIR + str(i + 1) + '-test_predict1' + '.png', pr_mask[:, :, 0])
        cv2.imwrite(SAVE_DIR + str(i + 1) + '-test_predict2' + '.png', pr_mask[:, :, 1])
        cv2.imwrite(SAVE_DIR + str(i + 1) + '-test_predict3' + '.png', pr_mask[:, :, 2])



if __name__ == '__main__':
    test_confidence_map()
