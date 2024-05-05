# 4k Tip Prediction

Pipeline to create a keypoint detection model of an instrument tip, instrument shaft, its shadow tip. The distance bewteen the tip and shadow can aid the operator by predicting the distance to the retinal surface. The shaft keypoint is used to help the autonomous lightguide reposition itself so the shadow is unobstructed by the instrument shaft.  

## Usage

1. Record 4k video(s) for the dataset
2. Run `extract_frames.py` to extract the video frames for segmentation
3. (Optional) run `rename.py` to relabel the frames if frames were deleted or merging datasets
4. Label dataset with programs such as labelme
5. Run `make_labeled_image.py` to crop the images and create a Gaussian confidence map for each point
6. Run `augmentation.py` to create augmentated images and organize all images to relative training, testing, and validation folders
7. Run `train_confidence_map.py` to train the keypoint detection model with output results displayed
8. Run `video_output_confidence_map.py` to apply the keypoint detection model to the designated video to assess performance

## Contact

For any questions or inquiries, feel free to contact me at christiantgyuen@gmail.com or https://www.linkedin.com/in/christian-yuen/.
