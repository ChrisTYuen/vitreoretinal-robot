import os
import shutil
from tools import Parameters
from PIL import Image
import json


def is_image(file_path):
    try:
        Image.open(file_path)
        return True
    except IOError:
        return False
    
def is_json(file_path):
    try:
        with open(file_path, 'r') as f:
            json.load(f)
        return True
    except ValueError:
        return False

def rename():
    """
    This function renames and copies a specified number of image files and its corresponding json files from one directory to another, giving the copied files
    new names based on a starting number and the order in which they're processed. Original images can be deleted and the copied 
    files will be in order with no gaps.
    """
    parameters = Parameters.Parameters()
    ids = sorted(os.listdir(parameters.extracted_frames_path))
    extracted_fps = [os.path.join(parameters.extracted_frames_path, img_id) for img_id in ids if is_image(os.path.join(parameters.extracted_frames_path, img_id))]

    if not os.path.exists(parameters.folder_path + 'workspace/' + parameters.model):
        os.mkdir(parameters.folder_path + 'workspace/' + parameters.model)
    if not os.path.exists(parameters.json_and_raw_path):
        os.mkdir(parameters.json_and_raw_path)

    image_number = 1  # Start numbering from 1

    for i in range(len(extracted_fps)):
        # Copy and rename image file
        shutil.copyfile(extracted_fps[i], os.path.join(parameters.json_and_raw_path, str(image_number) + '.png'))

        # Copy and rename corresponding JSON file
        json_file_path = extracted_fps[i].replace('.png', '.json')
        if os.path.exists(json_file_path) and is_json(json_file_path):
            shutil.copyfile(json_file_path, os.path.join(parameters.json_and_raw_path, str(image_number) + '.json'))

        image_number += 1  # Increment the image number for each file


if __name__ == '__main__':
    rename()
