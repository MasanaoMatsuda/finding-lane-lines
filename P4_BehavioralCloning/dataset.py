import os
import csv
import cv2
import numpy as np
import pandas as pd
import matplotlib.image as mpimg
import shutil
from pathlib import Path
from datetime import datetime
from keras.utils import Sequence


class Manipulator:

    def __init__(self, row_data_dir, input_data_dir):
        self.ROW_DATA_DIR = row_data_dir
        self.INPUT_DIR = input_data_dir
        self.INPUT_FILE_ONLY_ORIGIN = str(Path(self.INPUT_DIR) / 'all_origin.csv')
        self.INPUT_FILE_WITH_AUGMENT = str(Path(self.INPUT_DIR) / 'all_augment.csv')


    def load_input_dataset(self, augmentation, update):
        if update:
            if augmentation:
                self.assemble_input_data_with_augmentation()
            else:
                self.assemble_input_data_only_orign()

        if augmentation:
            path = self.INPUT_FILE_WITH_AUGMENT
        else:
            path = self.INPUT_FILE_ONLY_ORIGIN
        return pd.read_csv(path, names=("image_path", "steering"))


    def assemble_input_data_only_orign(self):
        output_file = os.path.abspath(self.INPUT_FILE_ONLY_ORIGIN)
        self.__reset_previous_assembly(output_file)

        for data in self.load_raw_data():
            data.to_csv(output_file, mode='a', header=False, index=False)


    def assemble_input_data_with_augmentation(self):
        output_file = os.path.abspath(self.INPUT_FILE_WITH_AUGMENT)
        self.__reset_previous_assembly(output_file)
        self.__reset_previous_augmentation()

        for data in self.load_raw_data():
            X_aug, y_aug = self.augment_flip_horizontaly(
                [mpimg.imread(path) for path in data[0].values],
                data[1].values)

            X_aug_path = self.save_images(os.path.abspath("./data/augment"), X_aug)
            data_aug = pd.DataFrame([X_aug_path, y_aug]).T

            data.to_csv(output_file, mode='a', header=False, index=False)
            data_aug.to_csv(output_file, mode='a', header=False, index=False)

    def __reset_previous_augmentation(self):
        aug_img_dir = os.path.abspath(str(Path(self.ROW_DATA_DIR) / 'augment'))
        if os.path.exists(aug_img_dir):
            shutil.rmtree(aug_img_dir)
        os.makedirs(aug_img_dir)


    def __reset_previous_assembly(self, output_file):
        if os.path.exists(output_file):
            os.remove(output_file)


    def load_raw_data(self):
        csvPaths = Path(self.ROW_DATA_DIR).glob('**\*.csv')
        for path in list(csvPaths):
            df = pd.read_csv(path,
                             names=("center", "left", "right", "steering", "throttle", "break", "speed"),
                             usecols=lambda col: col not in ["throttle", "break", "speed"])
            yield pd.DataFrame(df[["center", "steering"]].values)
            yield pd.DataFrame(df[["left", "steering"]].values)
            yield pd.DataFrame(df[["right", "steering"]].values)


    def augment_flip_horizontaly(self, X_data, y_data):
        assert len(X_data) == len(y_data) ,"Input parameter is invalid."
        X_aug, y_aug = [], []
        for x, y in zip(X_data, y_data):
            X_aug.append(x)
            y_aug.append(y)
        return X_aug, y_aug


    def save_images(self, save_dir, images):
        os.makedirs(os.path.abspath(save_dir), exist_ok=True)
        paths = []
        now = "{0:%Y%m%d_%M%S}".format(datetime.now())
        for idx, image in enumerate(images):
            path = str(Path(save_dir) / "aug_{}_{}.jpg".format(now, idx))
            paths.append(path)
            mpimg.imsave(path, image)
        return paths



class BatchSequence(Sequence):
    def __init__(self, X_data_path, y_data, batch_size):
        self.x = X_data_path
        self.y = y_data
        self.batch_size = batch_size

    def __getitem__(self, idx):
        batch_x_path = self.x[idx*self.batch_size: (idx+1)*self.batch_size]
        batch_x = [mpimg.imread(path) for path in batch_x_path]
        batch_y = self.y[idx*self.batch_size: (idx+1)*self.batch_size]
        return np.array(batch_x), np.array(batch_y)

    def __len__(self):
        return int(np.ceil(len(self.x) / float(self.batch_size)))

    def on_epoch_end(self):
        print("Shuffle")
        df = pd.DataFrame({"images": self.x, "steering": self.y})
        df = df.sample(frac=1)
        self.x = df["images"].values
        self.y = df["steering"].values
