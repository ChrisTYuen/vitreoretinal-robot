import os
import torch
import numpy as np
import cv2
from tools import functions
import segmentation_models_pytorch as smp
import time

RESULT_DIR = '/home/yuki/Documents/eye_surgery/image_processing/data_tip/result/classification_3_points/'

# load best saved checkpoint
best_model = torch.load(RESULT_DIR+'best_model.pth')
ENCODER = 'resnet34'
ENCODER_WEIGHTS = 'imagenet'
DEVICE = 'cuda'

preprocessing_fn = smp.encoders.get_preprocessing_fn(ENCODER, ENCODER_WEIGHTS)
preprocessing = functions.get_preprocessing(preprocessing_fn)

# VIDEO ABSOLUTE OR RELATIVE PATH
video_dir = '/home/yuki/Documents/eye_surgery/image_processing/data_tip/original/video/needle_current'
output_dir = RESULT_DIR + 'video/needle_current'
ids = os.listdir(video_dir)
print(ids)
video_fps = [os.path.join(video_dir, video_id) for video_id in ids]
output_fps = [os.path.join(output_dir, video_id) for video_id in ids]

kernel = np.ones((5, 5), np.uint8)
# print(kernel)


for video_number in range(len(video_fps)):
    print('Opening video...')
    cap = cv2.VideoCapture(video_fps[video_number])

    fps = int(cap.get(cv2.CAP_PROP_FPS))
    # w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    # h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    w = 1024
    h = 1024
    fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
    video = cv2.VideoWriter(output_fps[video_number], fourcc, fps, (512*2, 512))

    if cap.isOpened():
        total_frame = cap.get(cv2.CAP_PROP_FRAME_COUNT)
        print("Total Frame number is " + str(cap.get(cv2.CAP_PROP_FRAME_COUNT)))
    else:
        print("Video is not opened")
        break

    for i in range(int(total_frame)):
        # print(str(int((i + 1) / total_frame * 100)) + '%')
        if cap.isOpened():
            # print(i)
            ret, frame = cap.read()
            if ret:
                start = time.time()
                # print(ret)
                frame = cv2.resize(frame, (h, h))
                frame = frame[256:768, 256:768]
                frame_original = frame
                # print(np.shape(frame_original))
                frame = preprocessing(image=frame)['image']
                x_tensor = torch.from_numpy(frame).to(DEVICE).unsqueeze(0)
                pr_mask = best_model.predict(x_tensor)
                pr_mask = (pr_mask.squeeze().cpu().numpy().round())
                pr_mask = np.transpose(pr_mask, (1, 2, 0))

                # x1, y1 = functions.get_instrument_tip(pr_mask[:, :, 0])
                # x2, y2 = functions.get_shadow_tip(pr_mask[:, :, 1])

                pr_mask_output = np.ones(frame_original.shape) * 200

                for jj in range(3):
                    pr_mask_output[:, :, jj] = pr_mask_output[:, :, jj] - 200 * pr_mask[:, :, 0] \
                                               - 150 * pr_mask[:, :, 2] - 100 * pr_mask[:, :, 1]

                # cv2.circle(pr_mask_output, (x1, y1), 2, (0, 0, 255), -1)
                # cv2.circle(pr_mask_output, (x2, y2), 2, (255, 0, 0), -1)
                #
                # distance = int(np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2))
                # cv2.putText(pr_mask_output, str(distance) + ' pixel', (10, 40),
                #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                pr_mask_output = pr_mask_output.astype('uint8')
                output = cv2.hconcat([frame_original, pr_mask_output])
                # print(np.shape(output))
                # print(time.time()-start)

                video.write(output)

    print(str(output_fps[video_number]) + ' is completed!')

    video.release()
    # Release video
    cap.release()
    # Close windows
    # cv2.destroyAllWindows()