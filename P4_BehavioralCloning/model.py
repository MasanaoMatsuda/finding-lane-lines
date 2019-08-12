import os
import sys
import csv
import cv2
import numpy as np
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import warnings
from math import ceil
from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle
from keras.models import Sequential, Model
from keras.layers import Conv2D, Flatten, Dense, MaxPooling2D, Dropout, Activation, Lambda, Cropping2D
from keras.optimizers import Adam
from dataset import Manipulator, BatchSequence

warnings.filterwarnings('ignore')

update = input("Do you want to update the dataset? [y/n]")
augmentation = input("Do you need augmentation? [y/n]")
batch_size = int(input("Enter the number of batch_size: "))
epoch = int(input("Enter the number of EPOCH: "))
out_fname = input("Enter the name of output file (ex. XXXX.h5): ")

if update == 'y':
    isUpdate = True
else:
    isUpdate = False

if augmentation == 'y':
    addAugmentation = True
else:
    addAugmentation = False

print()
print("Please confirm..")
print("Update the dataset => {}".format(isUpdate))
print("Batch Size => {}".format(batch_size))
print("Epoch => {}".format(epoch))
print("Save model to => {}".format(out_fname))
print()

if input("Can I proceed? [y/n]") == 'y':
    print("Yep! Let's dive into learning!")
else:
    print("Please retry.")
    sys.exit()

manipulator = Manipulator("./data", './input_data')
all_data = manipulator.load_input_dataset(addAugmentation, isUpdate)

train_set, valid_set = train_test_split(all_data, test_size=0.2, shuffle=True)

model = Sequential()
model.add(Cropping2D(cropping=((50,20),(0,0)), input_shape=(160,320,3)))
model.add(Lambda(lambda x: (x / 255.0)-0.5))
model.add(Conv2D(filters=24, kernel_size=(5,5), strides=(2,2), activation='relu'))
model.add(Conv2D(filters=36, kernel_size=(5,5), strides=(2,2), activation='relu'))
model.add(Conv2D(filters=48, kernel_size=(3,3), strides=(2,2), activation='relu'))
model.add(Dropout(0.5))
model.add(Conv2D(filters=64, kernel_size=(3,3), activation='relu'))
model.add(Conv2D(filters=64, kernel_size=(3,3), activation='relu'))
model.add(Dropout(0.5))
model.add(Flatten())
model.add(Dense(100))
model.add(Dense(50))
model.add(Dense(10))
model.add(Dense(1))


model.compile(loss='mse', optimizer=Adam(lr=0.0001))
history_obj = model.fit_generator(BatchSequence(train_set["image_path"].values, train_set["steering"].values, batch_size),
                                  steps_per_epoch=ceil(len(train_set)/batch_size),
                                  validation_data=BatchSequence(valid_set["image_path"].values, valid_set["steering"].values, batch_size),
                                  validation_steps=ceil(len(valid_set)/batch_size),
                                  nb_epoch=epoch,
                                  verbose=1)
model.save(out_fname)

### plt.plot(history_obj.history['loss'])
plt.plot(history_obj.history['val_loss'])
plt.title('model mean squared error loss')
plt.ylabel('mean squared error loss')
plt.xlabel('epoch')
plt.legend(['training set', 'validation set'], loc='upper right')
plt.show()

model.summary()
