import cv2
import numpy as np

class AugmentGenerator:

    def __init__(self, ang_range, trans_range, shear_range):
        self.ang_range = ang_range
        self.trans_range = trans_range
        self.shear_range = shear_range

    def augment_rotation(self, image, ang_range):
        angle = np.random.uniform(ang_range*2)-ang_range
        rotate_M = cv2.getRotationMatrix2D((image.shape[0]/2, image.shape[1]/2), angle, 1)
        return cv2.warpAffine(image, rotate_M, (image.shape[0], image.shape[1]))

    def augment_translation(self, image, trans_range):
        trans = trans_range*np.random.uniform() - trans_range/2
        trans_M = np.float32([[1,0,trans], [0,1,trans]])
        return cv2.warpAffine(image, trans_M, (image.shape[0], image.shape[1]))

    def augment_shear(self, image, shear_range):
        pts1 = np.float32([[5,5],[20,5],[5,20]])
        pt1 = 5 + shear_range*np.random.uniform() - shear_range/2
        pt2 = 20 + shear_range*np.random.uniform() - shear_range/2
        pts2 = np.float32([[pt1,5],[pt2,pt1],[5,pt2]])
        shear_M = cv2.getAffineTransform(pts1,pts2)
        return cv2.warpAffine(image, shear_M, (image.shape[0], image.shape[1]))

    def augment_brightness(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        hsv[:,:,2] = hsv[:,:,2] * (np.random.uniform(0.25, 1.0))
        return cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)

    def transform_image(self, image):
        image = self.augment_rotation(image, self.ang_range)
        image = self.augment_translation(image, self.trans_range)
        image = self.augment_shear(image, self.shear_range)
        image = self.augment_brightness(image)
        return image

    def generate(self, images, labels, num_of_augment):
        aug_images = []
        aug_labels = []
        for idx, image in enumerate(images):
            for i in range(num_of_augment):
                aug_images.append(self.transform_image(image))
            aug_labels.extend([labels[idx]]*num_of_augment)
        return np.array(aug_images), np.array(aug_labels)
