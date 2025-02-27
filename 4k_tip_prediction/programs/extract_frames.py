import os
import cv2
import shutil
from tools import Parameters


def extract_frames():
    """
    This function extracts frames from video files at specified intervals and save them as images. The frames are saved in a directory 
    specified by the Parameters object, and the images are named in ascending order starting from the first extracted frame.
    """
    parameters = Parameters.Parameters()
    ids = os.listdir(parameters.video_path)
    ids.reverse()
    print(ids)

    frame_interval = parameters.frame_interval

    total_image_num = 0
    # Open Video
    print('Opening video...')
    for k in range(0, len(ids)):
        cap = cv2.VideoCapture(parameters.video_path + ids[k])
        if cap.isOpened():
            total_image_num = total_image_num + int(cap.get(cv2.CAP_PROP_FRAME_COUNT) / frame_interval + 1)
            print('Total frame count: ' + str(int(cap.get(cv2.CAP_PROP_FRAME_COUNT))))
        else:
            print("Video is not opened")

        cap.release()

    shutil.rmtree(parameters.extracted_frames_path)
    os.mkdir(parameters.extracted_frames_path)
    extracted_num = 0
    # extracted_num = len(os.listdir(parameters.extracted_frames_path))
    start_png_num = extracted_num
    print('Total output image number is ' + str(total_image_num) + ' images. Extracting frames...')

    total_image_num = 0
    for k in range(0, len(ids)):
        cap = cv2.VideoCapture(parameters.video_path + ids[k])
        if cap.isOpened():
            total_image_num = cap.get(cv2.CAP_PROP_FRAME_COUNT)
            # print("Total Frame number is " + str(cap.get(cv2.CAP_PROP_FRAME_COUNT)))
        else:
            print("Video is not opened")

        # Skip useless frames
        for i in range(0, int(total_image_num)):
            if cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    print(f"Failed to read frame {i}... Try redownloading or re-coding the video")
                    break
                if ret and i % frame_interval == 0:  # Check frame is read correctly
                    print(f"Processing frame {i}")
                    start_png_num = start_png_num + 1
                    # OUTPUT IMAGE ABSOLUTE OR RELATIVE PATH
                    output_image_path = parameters.extracted_frames_path + str(start_png_num) + '.png'
                    cv2.imwrite(output_image_path, frame)

        print(ids[k] + ' is completed!')
        # Release video
        cap.release()


if __name__ == '__main__':
    extract_frames()
