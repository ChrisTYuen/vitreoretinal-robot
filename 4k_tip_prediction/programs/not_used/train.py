import os
from tools.Dataset import Dataset
from tools import functions
import torch
from torch.utils.data import DataLoader
import segmentation_models_pytorch as smp
import matplotlib.pyplot as plt
import numpy as np
import cv2
import time

os.environ['CUDA_VISIBLE_DEVICES'] = '0'
if not torch.cuda.is_available():
    raise Exception("GPU not available. CPU training will be too slow.")
print("device name", torch.cuda.get_device_name(0))


DATA_DIR = '/home/yuki/Documents/eye_surgery/image_processing/data_tip/dataset/'
x_train_dir = os.path.join(DATA_DIR, 'train')
y_train_dir = os.path.join(DATA_DIR, 'trainannot')
x_valid_dir = os.path.join(DATA_DIR, 'val')
y_valid_dir = os.path.join(DATA_DIR, 'valannot')

RESULT_DIR = '/home/yuki/Documents/eye_surgery/image_processing/data_tip/result/classification_3_points/'

CLASSES = ['tip_instrument', 'tip_another_instrument', 'tip_shadow']
ENCODER = 'resnet34'
ENCODER_WEIGHTS = 'imagenet'
ACTIVATION = 'sigmoid'
DEVICE = 'cuda'

model = smp.Unet(encoder_name=ENCODER, encoder_weights=ENCODER_WEIGHTS, classes=len(CLASSES), activation=ACTIVATION)
# print(model)

# data normalization function
preprocessing_fn = smp.encoders.get_preprocessing_fn(ENCODER, ENCODER_WEIGHTS)
# print(preprocessing_fn)

train_dataset = Dataset(x_train_dir, y_train_dir, preprocessing=functions.get_preprocessing(preprocessing_fn),
                        classes=CLASSES)

valid_dataset = Dataset(x_valid_dir, y_valid_dir, preprocessing=functions.get_preprocessing(preprocessing_fn),
                        classes=CLASSES)

print('the number of image/label in the train: ', len(train_dataset))

train_loader = DataLoader(train_dataset, batch_size=5, shuffle=True, num_workers=12)
valid_loader = DataLoader(valid_dataset, batch_size=1, shuffle=False, num_workers=12)

loss = smp.utils.losses.DiceLoss()
metrics = [smp.utils.metrics.IoU(threshold=0.5)]
optimizer = torch.optim.Adam([dict(params=model.parameters(), lr=0.0001)])

# create epoch runners
# it is a simple loop of iterating over dataloader`s samples
train_epoch = smp.utils.train.TrainEpoch(model, loss=loss, metrics=metrics, optimizer=optimizer, device=DEVICE,
                                         verbose=True)
valid_epoch = smp.utils.train.ValidEpoch(model, loss=loss, metrics=metrics, device=DEVICE, verbose=True)

max_score = 0

x_epoch_data = []
train_dice_loss = []
train_iou_score = []
valid_dice_loss = []
valid_iou_score = []

train_dataset_vis = Dataset(x_train_dir, y_train_dir, classes=CLASSES)
valid_dataset_vis = Dataset(x_valid_dir, y_valid_dir, classes=CLASSES)

for i in range(0, 40):
    print('\nEpoch: {}'.format(i))
    train_logs = train_epoch.run(train_loader)
    valid_logs = valid_epoch.run(valid_loader)

    x_epoch_data.append(i)
    train_dice_loss.append(train_logs['dice_loss'])
    train_iou_score.append(train_logs['iou_score'])
    valid_dice_loss.append(valid_logs['dice_loss'])
    valid_iou_score.append(valid_logs['iou_score'])

    torch.save(model, RESULT_DIR+'current.pth')
    current_model = torch.load(RESULT_DIR+'current.pth')

    n_train = np.random.choice(len(train_dataset))
    n_valid = np.random.choice(len(valid_dataset))

    image_train, mask_train = train_dataset[n_train]
    image_vis_train = train_dataset_vis[n_train][0].astype('uint8')
    mask_vis_train = train_dataset_vis[n_train][1].astype('uint8')
    # print(mask_vis_train.shape)
    mask_gray_train = np.zeros((mask_vis_train.shape[0], mask_vis_train.shape[1]))
    for ii in range(mask_vis_train.shape[2]):
        mask_gray_train = mask_gray_train + 50 * ii * mask_vis_train[:, :, ii]
    # print(mask_vis_train)
    x_tensor = torch.from_numpy(image_train).to(DEVICE).unsqueeze(0)
    cv2.imwrite(RESULT_DIR+'process_image/' + str(i+1) + '-train' + '.png', image_vis_train)
    cv2.imwrite(RESULT_DIR+'process_image/' + str(i + 1) + '-train_mask' + '.png', mask_gray_train)
    pr_mask_train = current_model.predict(x_tensor)
    pr_mask_train = (pr_mask_train.squeeze().cpu().numpy().round())
    pr_mask_train = np.transpose(pr_mask_train, (1, 2, 0))
    pr_mask_gray_train = np.zeros((pr_mask_train.shape[0], pr_mask_train.shape[1]))
    for ii in range(pr_mask_train.shape[2]):
        pr_mask_gray_train = pr_mask_gray_train + 50 * ii * pr_mask_train[:, :, ii]
    cv2.imwrite(RESULT_DIR+'process_image/' + str(i + 1) + '-train_predict' + '.png', pr_mask_gray_train)

    image_valid, mask_valid = valid_dataset[n_valid]
    image_vis_valid = valid_dataset_vis[n_valid][0].astype('uint8')
    mask_vis_valid = valid_dataset_vis[n_valid][1].astype('uint8')
    mask_gray_valid = np.zeros((mask_vis_valid.shape[0], mask_vis_valid.shape[1]))
    for ii in range(mask_vis_valid.shape[2]):
        mask_gray_valid = mask_gray_valid + 50 * ii * mask_vis_valid[:, :, ii]
    x_tensor = torch.from_numpy(image_valid).to(DEVICE).unsqueeze(0)
    cv2.imwrite(RESULT_DIR+'process_image/' + str(i + 1) + '-valid' + '.png', image_vis_valid)
    cv2.imwrite(RESULT_DIR+'process_image/' + str(i + 1) + '-valid_mask' + '.png', mask_gray_valid)
    pr_mask_valid = current_model.predict(x_tensor)
    pr_mask_valid = (pr_mask_valid.squeeze().cpu().numpy().round())
    pr_mask_valid = np.transpose(pr_mask_valid, (1, 2, 0))
    pr_mask_gray_valid = np.zeros((pr_mask_valid.shape[0], pr_mask_valid.shape[1]))
    for ii in range(pr_mask_valid.shape[2]):
        pr_mask_gray_valid = pr_mask_gray_valid + 50 * ii * pr_mask_valid[:, :, ii]
    cv2.imwrite(RESULT_DIR+'process_image/' + str(i + 1) + 'valid_predict' + '.png', pr_mask_gray_valid)

    # do something (save model, change lr, etc.)
    if max_score < valid_logs['iou_score']:
        max_score = valid_logs['iou_score']
        torch.save(model, RESULT_DIR+'best_model.pth')
        print('Model saved!')

    if i == 30:
        optimizer.param_groups[0]['lr'] = 1e-5
        print('Decrease decoder learning rate to 1e-5!')


fig = plt.figure(figsize=(14, 5))
ax1 = fig.add_subplot(1, 2, 1)
line1, = ax1.plot(x_epoch_data, train_dice_loss, label='train')
line2, = ax1.plot(x_epoch_data, valid_dice_loss, label='validation')
ax1.set_title("dice loss")
ax1.set_xlabel('epoch')
ax1.set_ylabel('dice_loss')
ax1.legend(loc='upper right')

ax2 = fig.add_subplot(1, 2, 2)
line1, = ax2.plot(x_epoch_data, train_iou_score, label='train')
line2, = ax2.plot(x_epoch_data, valid_iou_score, label='validation')
ax2.set_title("iou score")
ax2.set_xlabel('epoch')
ax2.set_ylabel('iou_score')
ax2.legend(loc='upper left')

plt.show()
