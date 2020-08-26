from keras import optimizers
import numpy as np
import Model
import DataProcessing

if __name__ == '__main__':
    train_dir = './Train_Data'
    train_imgs = DataProcessing.load_images(train_dir)

    pose = np.load('./Train_Data/pose.npy')

    model = Model.create_cnn(200, 200, 3)
    adam = optimizers.Adam()
    model.compile(loss='mean_squared_error', optimizer=adam)
    model.summary()
