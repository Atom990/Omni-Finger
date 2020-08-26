import cv2
import os
import glob
import numpy as np
import re


def get_index(s):
    s = re.split('_|.jpg', s)
    return int(s[5])


def sort_files(name_list):
    return sorted(name_list, key=get_index)


def load_images(path):
    path = glob.glob(os.path.join(path, '*.jpg'))
    path = sort_files(path)
    output_images = []
    for pic in path:
        image = cv2.imread(pic)
        image = cv2.resize(image, (200, 200))
        output_images.append(image)
    output_images = np.array(output_images) / 255
    return output_images

# path = glob.glob(os.path.join('./Train_Data', '*.jpg'))
# sorted_path = sort_files(path)
# print()
