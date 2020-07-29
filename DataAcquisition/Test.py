import cv2
import numpy as np
import FTReading

cam = cv2.VideoCapture(2)
ret, picture = cam.read()
i = 100
cv2.imwrite('./Pictures/{index}.jpg'.format(index=i), picture)
