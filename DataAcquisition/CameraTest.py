import cv2

cam = cv2.VideoCapture(2)
img_counter = 0
while cam.isOpened():
    ret, frame = cam.read()
    cv2.imshow("test", frame)
    if not ret:
        break
    key = cv2.waitKey(1) & 0xFF

    if key == 27:
        # press ESC to escape (ESC ASCII value: 27)
        print("Escape hit, closing...")
        break
    elif key == 32:
        # press Space to capture image (Space ASCII value: 32)
        img_name = "opencv_frame_{}.png".format(img_counter)
        cv2.imwrite(img_name, frame)
        print("{} written!".format(img_name))
        img_counter += 1
    else:
        pass
cam.release()
cv2.destroyAllWindows()
