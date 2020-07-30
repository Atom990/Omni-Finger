import cv2
import numpy as np
import FTReading

press_info_data = []
D = 0.005
d = 0.01
pose_finger = np.array([1, 2, 3, 4, 5, 6])
pose = pose_finger[:3]
a = pose_finger[3]
press_info_data.append(np.hstack((D, d, pose_finger[0:2])))
press_info_data = np.array(press_info_data)
print(press_info_data)
