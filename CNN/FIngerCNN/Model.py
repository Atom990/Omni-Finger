from keras import layers
from keras import models
from keras.layers import LeakyReLU


def create_cnn(width, height, depth, filters=(32, 64, 64, 128)):
    model = models.Sequential()

    for (i, f) in enumerate(filters):

        if i == 0:
            model.add(layers.Conv2D(f, (3, 3), activation='relu', input_shape=(width, height, depth)))
        else:
            model.add(layers.Conv2D(f, (3, 3), ))

        model.add(layers.MaxPooling2D(2, 2))

    model.add(layers.Flatten())
    model.add(layers.Dense(512, activation='relu'))
    model.add(layers.Dense(6))

    return model
