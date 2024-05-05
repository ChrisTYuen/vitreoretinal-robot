import os
import torch
import numpy as np
import cv2
from tools import functions
import segmentation_models_pytorch as smp
import time
from tools import Parameters
from tools import functions as fn


def video_output_confidence_map():
    """
    This program uses a pre-trained model to perform keypoint detection on a set of videos. For each video, it reads the frames,
    selects a region of interest (ROI), and uses the model to predict keypoints in the ROI. It then draws the detected keypoints and the 
    ROI on the frame, resizes the frame, and writes it to an output video. The program also adjusts the ROI based on the maximum displacement 
    of the keypoints from the previous frame. The process is repeated for all frames in all videos in the specified directory.
    """
    parameters = Parameters.Parameters()
    RESULT_DIR = parameters.result_path

    predict_size = parameters.predict_size

    best_model = torch.load(RESULT_DIR + 'best_model.pth')
    print("loaded!")
    ENCODER = parameters.ENCODER
    ENCODER_WEIGHTS = parameters.ENCODER_WEIGHTS

    preprocessing_fn = smp.encoders.get_preprocessing_fn(ENCODER, ENCODER_WEIGHTS)
    preprocessing = functions.get_preprocessing(preprocessing_fn)

    # VIDEO ABSOLUTE OR RELATIVE PATH
    video_dir = parameters.video_path
    if not os.path.exists(RESULT_DIR + 'video/'):
        os.mkdir(RESULT_DIR + 'video/')
    output_dir = RESULT_DIR + 'video/'
    ids = os.listdir(video_dir)
    video_fps = [os.path.join(video_dir, video_id) for video_id in ids]
    output_fps = [os.path.join(output_dir, video_id) for video_id in ids]

    for video_number in range(len(video_fps)):
        print('Opening video...')
        cap = cv2.VideoCapture(video_fps[video_number])

        fps = int(cap.get(cv2.CAP_PROP_FPS))
        original_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        original_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        ROI_center_initial_x = original_w / 4
        ROI_center_initial_y = original_h * 3 / 4

        fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
        video = cv2.VideoWriter(output_fps[video_number], fourcc, fps,
                                (predict_size + int(original_w / original_h * predict_size), predict_size))

        if cap.isOpened():
            total_frame = cap.get(cv2.CAP_PROP_FRAME_COUNT)
            print("Total Frame number is " + str(cap.get(cv2.CAP_PROP_FRAME_COUNT)))
        else:
            print("Video is not opened")
            break

        ROI_center = np.array([ROI_center_initial_x, ROI_center_initial_y])

        threshold1 = parameters.threshold1
        threshold2 = parameters.threshold2
        threshold3 = parameters.threshold3

        max_dis = threshold1/2

        for i in range(int(total_frame)):
            print(str(int(i/total_frame*100)) + ' %')
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    start = time.time()

                    if max_dis + parameters.margin <= threshold1 / 2:
                        ROI_dis = threshold1 / 2
                    elif max_dis + parameters.margin <= threshold2 / 2:
                        ROI_dis = threshold2 / 2
                    else:
                        ROI_dis = threshold3 / 2

                    if ROI_center[0] - ROI_dis < 0:
                        ROI_center[0] = ROI_dis
                    if ROI_center[1] - ROI_dis < 0:
                        ROI_center[1] = ROI_dis
                    if ROI_center[0] + ROI_dis >= original_w:
                        ROI_center[0] = original_w - ROI_dis
                    if ROI_center[1] + ROI_dis >= original_h:
                        ROI_center[1] = original_h - ROI_dis

                    ROI_center = ROI_center.astype('uint64')
                    left_top = ROI_center - np.array([ROI_dis, ROI_dis])
                    left_top = left_top.astype('uint64')
                    right_bottom = left_top + np.array([2 * ROI_dis, 2 * ROI_dis])
                    right_bottom = right_bottom.astype('uint64')

                    frame_interested = frame[left_top[1]:right_bottom[1], left_top[0]:right_bottom[0]]
                    frame_interested = cv2.resize(frame_interested, (predict_size, predict_size))

                    if parameters.print_center:
                        cv2.circle(frame, (int(ROI_center[0]), int(ROI_center[1])), 50, (0, 255, 0), -1)

                    ROI_img, ROI_center, max_dis = fn.predict_and_mark(frame_interested, left_top, best_model,
                                                                       preprocessing, parameters, ROI_dis, i)

                    cv2.rectangle(frame, (left_top[0], left_top[1]), (right_bottom[0], right_bottom[1]),
                                  (0, 255, 0), thickness=5)
                    if parameters.print_center:
                        cv2.circle(frame, (int(ROI_center[0]), int(ROI_center[1])), 50, (0, 0, 255), -1)
                    frame_resize = cv2.resize(frame, (int(original_w / original_h * predict_size), predict_size))

                    output = cv2.hconcat([frame_resize, ROI_img])
                    if parameters.print_time:
                        print(time.time() - start)

                    # cv2.imshow('img_test', output)
                    # cv2.waitKey(1)

                    video.write(output)

        print(str(output_fps[video_number]) + ' is completed!')

        video.release()
        # Release video
        cap.release()
        # Close windows
        # cv2.destroyAllWindows()


if __name__ == '__main__':
    video_output_confidence_map()
