import os
import torch
import numpy as np
import cv2
import segmentation_models_pytorch as smp
import time
from tools.Parameters import Parameters
from tools import functions as fn


def video_output_confidence_map():
    """
    This program uses a pre-trained model to perform keypoint detection on a set of videos. For each video, it reads the frames,
    selects a region of interest (ROI), and uses the model to predict keypoints in the ROI. It then draws the detected keypoints and the 
    ROI on the frame, resizes the frame, and writes it to an output video. The program also adjusts the ROI based on the maximum displacement 
    of the keypoints from the previous frame. The process is repeated for all frames in all videos in the specified directory.
    """
    RESULT_DIR = Parameters.result_path
    ROI_DIR = Parameters.ROI_path

    predict_size = Parameters.predict_size
    output_size = Parameters.output_size

    best_model = torch.load(RESULT_DIR + 'best_model.pth')
    ROI_model = torch.load(ROI_DIR + 'best_model.pth')
    print("loaded!")
    ENCODER = Parameters.ENCODER
    ENCODER_WEIGHTS = Parameters.ENCODER_WEIGHTS

    preprocessing_fn = smp.encoders.get_preprocessing_fn(ENCODER, ENCODER_WEIGHTS)
    preprocessing = fn.get_preprocessing(preprocessing_fn)

    # VIDEO ABSOLUTE OR RELATIVE PATH
    video_dir = Parameters.video_path
    if not os.path.exists(RESULT_DIR + 'video/'):
        os.mkdir(RESULT_DIR + 'video/')
    output_dir = RESULT_DIR + 'video/'
    ids = os.listdir(video_dir)
    video_fps = [os.path.join(video_dir, video_id) for video_id in ids]
    output_fps = [os.path.join(output_dir, video_id) for video_id in ids]

    ROI = fn.ROIPredictor(ROI_model, preprocessing, Parameters)
    detect = fn.Detector(best_model, preprocessing, Parameters)    

    for video_number in range(len(video_fps)):
        print('Opening video...')
        cap = cv2.VideoCapture(video_fps[video_number])

        fps = int(cap.get(cv2.CAP_PROP_FPS))
        original_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        original_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
        video = cv2.VideoWriter(output_fps[video_number], fourcc, fps,
                                (output_size + int(original_width / original_height * output_size), output_size))

        if cap.isOpened():
            total_frame = cap.get(cv2.CAP_PROP_FRAME_COUNT)
            print("Total Frame number is " + str(cap.get(cv2.CAP_PROP_FRAME_COUNT)))
        else:
            print("Video is not opened")
            break

        # size of the ROI
        threshold1 = Parameters.threshold1
        threshold2 = Parameters.threshold2
        threshold3 = Parameters.threshold3

        # start video processing
        last_percent = 0
        for i in range(int(total_frame)):
            percent = int(i/total_frame*100)
            if percent != last_percent:
                print(str(percent) + ' %')
                last_percent = percent
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    start = time.time()

                    # Get the ROI center and distance between the instrument tip and the shadow
                    ROI_center, max_dis, point_instrument  = ROI.predict_ROI(frame, original_width, original_height)

                    # Check if the keypoints are are within the ROI and adjust the size of the ROI if necessary
                    if max_dis + Parameters.margin <= threshold1 / 2:
                        ROI_dis = threshold1 / 2
                    elif max_dis + Parameters.margin <= threshold2 / 2:
                        ROI_dis = threshold2 / 2
                    else:
                        ROI_dis = threshold3 / 2

                    if ROI_center[0] - ROI_dis < 0:
                        ROI_center[0] = ROI_dis
                    if ROI_center[1] - ROI_dis < 0:
                        ROI_center[1] = ROI_dis
                    if ROI_center[0] + ROI_dis >= original_width:
                        ROI_center[0] = original_width - ROI_dis
                    if ROI_center[1] + ROI_dis >= original_height:
                        ROI_center[1] = original_height - ROI_dis

                    # Convert the ROI center and distance to integers and calculate the left top and right bottom corners of the ROI
                    ROI_center = ROI_center.astype('uint64')
                    left_top = (ROI_center - np.array([ROI_dis, ROI_dis])).astype('uint64')
                    right_bottom = (left_top + np.array([2 * ROI_dis, 2 * ROI_dis])).astype('uint64')
                    frame_interested = frame[left_top[1]:right_bottom[1], left_top[0]:right_bottom[0]]
                    
                    # Calculate the point_instrument position relative to the ROI frame
                    ratio = ROI_dis / predict_size * 2
                    tip_instrument_scaled = point_instrument - left_top
                    point_inst_in_ROI = np.array([tip_instrument_scaled[1] / ratio, tip_instrument_scaled[0] / ratio])

                    # Preprocess the frame image
                    preprocessed_frame_interested = fn.preprocess_image(frame_interested)

                    if Parameters.print_center:
                        cv2.circle(frame, (int(ROI_center[0]), int(ROI_center[1])), 50, (0, 255, 0), -1)

                    # Predict the keypoints and draw them on the frame
                    ROI_img = detect.predict_and_mark(ROI_dis, preprocessed_frame_interested, frame_interested, point_inst_in_ROI, i)

                    # Draw the ROI on the frame
                    cv2.rectangle(frame, (left_top[0], left_top[1]), (right_bottom[0], right_bottom[1]),
                                  (0, 255, 0), thickness=5)
                    cv2.circle(frame, (int(point_instrument[0]), int(point_instrument[1])), 10, (0, 255, 0), -1)
                    if Parameters.print_center:
                        cv2.circle(frame, (int(ROI_center[0]), int(ROI_center[1])), 50, (0, 0, 255), -1)
                    frame_resize = cv2.resize(frame, (int(original_width / original_height * output_size), output_size))

                    output = cv2.hconcat([frame_resize, ROI_img])
                    if Parameters.print_time:
                        print(time.time() - start)

                    # Display the frame while processing
                    cv2.imshow('img_test', output)
                    cv2.waitKey(1)

                    video.write(output)

        print(str(output_fps[video_number]) + ' is completed!')

        video.release()
        # Release video
        cap.release()
        # Close windows
        # cv2.destroyAllWindows()


if __name__ == '__main__':
    video_output_confidence_map()
