import cv2

cam = cv2.VideoCapture(2)
for i in range(0, 20):
    ret, pic = cam.read()
    cv2.imwrite('./Data/{name}.jpg'.format(name=i), pic)
