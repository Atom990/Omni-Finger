from keras import models
import DataProcessing
import numpy as np
import matplotlib.pyplot as plt

model = models.load_model('test_cnn')

validation_dir = '../../Data/Validation_Data'

validation_imgs = DataProcessing.load_images(validation_dir)
validation_pose = np.load('../../Data/Validation_Data/pose.npy')

fx = validation_pose[:, 0]
fy = validation_pose[:, 1]
fz = validation_pose[:, 2]
tx = validation_pose[:, 3]
ty = validation_pose[:, 4]
tz = validation_pose[:, 5]

pred = model.predict(validation_imgs)

pred_fx = pred[:, 0]
pred_fy = pred[:, 1]
pred_fz = pred[:, 2]
pred_tx = pred[:, 3]
pred_ty = pred[:, 4]
pred_tz = pred[:, 5]

err_fx = np.abs(pred_fx - fx)
err_fy = np.abs(pred_fy - fy)
err_fz = np.abs(pred_fz - fz)
err_tx = np.abs(pred_tx - tx)
err_ty = np.abs(pred_ty - ty)
err_tz = np.abs(pred_tz - tz)

plt.plot(err_fx)
plt.show()
