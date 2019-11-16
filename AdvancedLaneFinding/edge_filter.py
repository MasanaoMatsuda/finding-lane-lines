import cv2
import numpy as np

class Filter:
    def sobel(img, orientation='x'):
        img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        if orientation == 'x':
            sobel = cv2.Sobel(img_gray, cv2.CV_64F, 1, 0)
        elif orientation == 'y':
            sobel = cv2.Sobel(img_gray, cv2.CV_64F, 0, 1)
        else:
            print("Unknown value of orientation.")
            return None
            abs_img = np.absolute(sobel)
            return abs_img * 255 / np.max(abs_img)

    def sobel_gradient_magnitude(img, ksize=3):
        img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        sobelx = cv2.Sobel(img_gray, cv2.CV_64F, 1, 0, ksize=ksize)
        sobely = cv2.Sobel(img_gray, cv2.CV_64F, 0, 1, ksize=ksize)
        grad_mag = np.sqrt(sobelx**2 + sobely**2)
        return grad_mag * 255 / np.max(grad_mag)

    def gradient_direction(img, ksize=3):
        img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        abs_sobelx = np.absolute(cv2.Sobel(img_gray, cv2.CV_64F, 1, 0, ksize=ksize))
        abs_sobely = np.absolute(cv2.Sobel(img_gray, cv2.CV_64F, 0, 1, ksize=ksize))
        return np.arctan2(abs_sobely, abs_sobelx)

    def hls(rgb_img, channel='s'):
        if channel == 'h':
            c = 0
        elif channel == 'l':
            c = 1
        elif channel == 's':
            c = 2
        else:
            print("Undefined parameter channel")
        hls = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2HLS)
        return hls[:,:,c]

    def create_binary_image(img, thresh_min=50, thresh_max=100):
        binary = np.zeros_like(img)
        binary[(img > thresh_min) & (img < thresh_max)] = 1
        return binary
