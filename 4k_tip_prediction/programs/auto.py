import extract_frames
import rename
import make_labeled_image
import augmentation
import train_confidence_map
import video_output_confidence_map

# extract_frames.extract_frames()
# rename.rename()
make_labeled_image.make_labeled_image()
augmentation.augmentation()
train_confidence_map.train_confidence_map()
video_output_confidence_map.video_output_confidence_map()
