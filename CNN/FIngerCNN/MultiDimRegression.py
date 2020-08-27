from keras import optimizers
import numpy as np
import Model
import DataProcessing
import matplotlib.pyplot as plt

if __name__ == '__main__':
    # read data used for training and validation
    train_dir = '../../Data/Train_Data'
    validation_dir = '../../Data/Validation_Data'

    train_imgs = DataProcessing.load_images(train_dir)
    validation_imgs = DataProcessing.load_images(validation_dir)

    train_pose = np.load('../../Data/Train_Data/pose.npy')
    validation_pose = np.load('../../Data/Validation_Data/pose.npy')

    model = Model.create_cnn(200, 200, 3)
    adam = optimizers.Adam()
    model.compile(loss='mean_squared_error', optimizer=adam)

    history = model.fit(x=train_imgs, y=train_pose,
                        validation_data=(validation_imgs, validation_pose), epochs=200, batch_size=40)
    model.save('test_cnn')

    # 绘制训练 & 验证的准确率值
    plt.plot(history.history['acc'])
    plt.plot(history.history['val_acc'])
    plt.title('Model accuracy')
    plt.ylabel('Accuracy')
    plt.xlabel('Epoch')
    plt.legend(['Train', 'Test'], loc='upper left')
    plt.show()

    # 绘制训练 & 验证的损失值
    plt.plot(history.history['loss'])
    plt.plot(history.history['val_loss'])
    plt.title('Model loss')
    plt.ylabel('Loss')
    plt.xlabel('Epoch')
    plt.legend(['Train', 'Test'], loc='upper left')
    plt.show()
