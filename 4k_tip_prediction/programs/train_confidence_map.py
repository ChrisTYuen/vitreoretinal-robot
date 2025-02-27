import os
from tools.Dataset_confidence_map import Dataset
from tools import functions
import torch
from torch.utils.data import DataLoader
import segmentation_models_pytorch as smp
import matplotlib.pyplot as plt
import numpy as np
import cv2
from tools.Parameters import Parameters
import shutil


def apply_activation(output, activation):
    if activation == 'softmax':
        return torch.softmax(output, dim=1)
    elif activation == 'sigmoid':
        return torch.sigmoid(output)
    else:
        return output


def train_confidence_map():
    image_size = Parameters.output_size
    CLASSES = Parameters.CLASSES
    ENCODER = Parameters.ENCODER
    ENCODER_WEIGHTS = Parameters.ENCODER_WEIGHTS
    ACTIVATION = Parameters.ACTIVATION
    DEVICE = Parameters.DEVICE

    DATA_DIR = Parameters.dataset_path
    RESULT_DIR = Parameters.result_path
    x_train_dir = os.path.join(DATA_DIR, 'train')
    y_train_dir = os.path.join(DATA_DIR, 'train_mask')
    x_valid_dir = os.path.join(DATA_DIR, 'val')
    y_valid_dir = os.path.join(DATA_DIR, 'val_mask')

    os.environ['CUDA_VISIBLE_DEVICES'] = '0'
    if not torch.cuda.is_available():
        raise Exception("GPU not available. CPU training will be too slow.")
    print("device name", torch.cuda.get_device_name(0))

    model = smp.Unet(encoder_name=ENCODER, encoder_weights=ENCODER_WEIGHTS, classes=len(CLASSES), activation=None)

    # data normalization function
    preprocessing_fn = smp.encoders.get_preprocessing_fn(ENCODER, ENCODER_WEIGHTS)

    train_dataset = Dataset(x_train_dir, y_train_dir, image_size,
                            preprocessing=functions.get_preprocessing(preprocessing_fn),
                            classes=CLASSES)

    valid_dataset = Dataset(x_valid_dir, y_valid_dir, image_size,
                            preprocessing=functions.get_preprocessing(preprocessing_fn),
                            classes=CLASSES)

    print('the number of image/label in the train: ', len(train_dataset))

    train_loader = DataLoader(train_dataset, batch_size=Parameters.train_batch_size, shuffle=True, num_workers=8)
    valid_loader = DataLoader(valid_dataset, batch_size=1, shuffle=False, num_workers=8)

    loss = smp.utils.losses.MSELoss()
    metrics = [smp.utils.metrics.IoU(threshold=0.5)]
    optimizer = torch.optim.Adam([dict(params=model.parameters(), lr=Parameters.lr_initial)], weight_decay=Parameters.weight_decay)

    # create epoch runners
    # it is a simple loop of iterating over dataloader`s samples
    train_epoch = smp.utils.train.TrainEpoch(model, loss=loss, metrics=metrics, optimizer=optimizer, device=DEVICE,
                                             verbose=True)
    valid_epoch = smp.utils.train.ValidEpoch(model, loss=loss, metrics=metrics, device=DEVICE, verbose=True)

    max_score = 1

    x_epoch_data = []
    train_mse_loss = []
    valid_mse_loss = []
    train_iou_scores = []
    valid_iou_scores = []

    train_dataset_vis = Dataset(x_train_dir, y_train_dir, image_size, classes=CLASSES)
    valid_dataset_vis = Dataset(x_valid_dir, y_valid_dir, image_size, classes=CLASSES)

    # shutil.rmtree(RESULT_DIR + 'process_image')
    if not os.path.exists(Parameters.folder_path + 'result/' + Parameters.model):
        os.mkdir(Parameters.folder_path + 'result/' + Parameters.model)
    if not os.path.exists(Parameters.folder_path + 'result/' + Parameters.model + '/' + Parameters.image_quality):
        os.mkdir(Parameters.folder_path + 'result/' + Parameters.model + '/' + Parameters.image_quality)
    if not os.path.exists(RESULT_DIR + 'process_image'):
        os.mkdir(RESULT_DIR + 'process_image')

    for i in range(0, Parameters.epoch):
        print('\nEpoch: {}'.format(i))
        train_logs = train_epoch.run(train_loader)
        valid_logs = valid_epoch.run(valid_loader)

        x_epoch_data.append(i)
        train_mse_loss.append(train_logs['mse_loss'])
        valid_mse_loss.append(valid_logs['mse_loss'])
        train_iou_scores.append(train_logs['iou_score'])
        valid_iou_scores.append(valid_logs['iou_score'])

        torch.save(model, RESULT_DIR + 'current.pth')
        current_model = torch.load(RESULT_DIR + 'current.pth')

        n_train = np.random.choice(len(train_dataset))
        n_valid = np.random.choice(len(valid_dataset))

        image_train, mask_train = train_dataset[n_train]
        image_vis_train = train_dataset_vis[n_train][0].astype('uint8')
        mask_vis_train = train_dataset_vis[n_train][1]

        cv2.imwrite(RESULT_DIR + 'process_image/' + str(i + 1) + '-train' + '.png', image_vis_train)
        cv2.imwrite(RESULT_DIR + 'process_image/' + str(i + 1) + '-train_mask1' + '.png', mask_vis_train[:, :, 0] * 255)
        cv2.imwrite(RESULT_DIR + 'process_image/' + str(i + 1) + '-train_mask2' + '.png', mask_vis_train[:, :, 1] * 255)
        if Parameters.train_other_point:
            cv2.imwrite(RESULT_DIR + 'process_image/' + str(i + 1) + '-train_mask3' + '.png', mask_vis_train[:, :, 2] * 255)

        x_tensor = torch.from_numpy(image_train).to(DEVICE).unsqueeze(0)
        pr_mask_train = model(x_tensor)  # Direct model call
        pr_mask_train = apply_activation(pr_mask_train, ACTIVATION)  # Apply activation
        pr_mask_train = (pr_mask_train.squeeze().cpu().detach().numpy() * 255).astype(np.uint8)
        pr_mask_train = np.transpose(pr_mask_train, (1, 2, 0))
        if Parameters.resize_input:
            pr_mask_train = cv2.resize(pr_mask_train, (Parameters.output_size, Parameters.output_size))

        cv2.imwrite(RESULT_DIR + 'process_image/' + str(i + 1) + '-train_predict1' + '.png', pr_mask_train[:, :, 0])
        cv2.imwrite(RESULT_DIR + 'process_image/' + str(i + 1) + '-train_predict2' + '.png', pr_mask_train[:, :, 1])
        if Parameters.train_other_point:
            cv2.imwrite(RESULT_DIR + 'process_image/' + str(i + 1) + '-train_predict3' + '.png', pr_mask_train[:, :, 2])

        image_valid, mask_valid = valid_dataset[n_valid]
        image_vis_valid = valid_dataset_vis[n_valid][0].astype('uint8')
        mask_vis_valid = valid_dataset_vis[n_valid][1]

        cv2.imwrite(RESULT_DIR + 'process_image/' + str(i + 1) + '-valid' + '.png', image_vis_valid)
        cv2.imwrite(RESULT_DIR + 'process_image/' + str(i + 1) + '-valid_mask1' + '.png', mask_vis_valid[:, :, 0] * 255)
        cv2.imwrite(RESULT_DIR + 'process_image/' + str(i + 1) + '-valid_mask2' + '.png', mask_vis_valid[:, :, 1] * 255)
        if Parameters.train_other_point:
            cv2.imwrite(RESULT_DIR + 'process_image/' + str(i + 1) + '-valid_mask3' + '.png', mask_vis_valid[:, :, 2] * 255)

        x_tensor = torch.from_numpy(image_valid).to(DEVICE).unsqueeze(0)
        pr_mask_valid = model(x_tensor)  # Direct model call
        pr_mask_valid = apply_activation(pr_mask_valid, ACTIVATION)  # Apply activation
        pr_mask_valid = (pr_mask_valid.squeeze().cpu().detach().numpy() * 255).astype(np.uint8)
        pr_mask_valid = np.transpose(pr_mask_valid, (1, 2, 0))
        if Parameters.resize_input:
            pr_mask_valid = cv2.resize(pr_mask_valid, (Parameters.output_size, Parameters.output_size))

        cv2.imwrite(RESULT_DIR + 'process_image/' + str(i + 1) + 'valid_predict1' + '.png', pr_mask_valid[:, :, 0])
        cv2.imwrite(RESULT_DIR + 'process_image/' + str(i + 1) + 'valid_predict2' + '.png', pr_mask_valid[:, :, 1])
        if Parameters.train_other_point:
            cv2.imwrite(RESULT_DIR + 'process_image/' + str(i + 1) + 'valid_predict3' + '.png', pr_mask_valid[:, :, 2])

        # do something (save model, change lr, etc.)
        if max_score > valid_logs['mse_loss']:
            max_score = valid_logs['mse_loss']
            torch.save(model, RESULT_DIR + 'best_model.pth')
            print('Model saved!')

        if i == Parameters.lr_change_epoch1:
            optimizer.param_groups[0]['lr'] = Parameters.lr1
            print('Decrease decoder learning rate to', Parameters.lr1)

        if i == Parameters.lr_change_epoch2:
            optimizer.param_groups[0]['lr'] = Parameters.lr2 
            print('Decrease decoder learning rate to', Parameters.lr2)

    # Plot MSE loss and IoU scores
    fig = plt.figure(figsize=(7, 5))
    plt.yscale('log')
    plt.plot(x_epoch_data, train_mse_loss, label='train')
    plt.plot(x_epoch_data, valid_mse_loss, label='validation')
    fig.suptitle("MSE loss", size="xx-large", color="blue", weight="bold")
    fig.supxlabel('epoch')
    fig.supylabel('MSE_loss')
    plt.legend(loc='upper right')

    plt.savefig(RESULT_DIR + 'learning_curve.png')

    fig = plt.figure(figsize=(7, 5))
    plt.plot(x_epoch_data, train_iou_scores, label='train')
    plt.plot(x_epoch_data, valid_iou_scores, label='validation')
    fig.suptitle("IoU scores", size="xx-large", color="green", weight="bold")
    fig.supxlabel('epoch')
    fig.supylabel('IoU score')
    plt.legend(loc='upper right')

    plt.savefig(RESULT_DIR + 'iou_scores.png')

    # plt.show()


if __name__ == '__main__':
    train_confidence_map()
