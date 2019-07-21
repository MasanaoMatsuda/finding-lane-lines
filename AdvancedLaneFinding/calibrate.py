import numpy as np
import cv2
import matplotlib.image as mpimg

class Calibration:
    def __init__(self, fnames, pattern_size, criteria):
        self.__calibrate_chessboard(fnames, pattern_size, criteria)

    @property
    def ret(self):
        return self.__ret
    @ret.setter
    def ret(self, value):
        self.__ret = value

    @property
    def cam_mtx(self):
        return self.__cam_mtx
    @cam_mtx.setter
    def cam_mtx(self, value):
        self.__cam_mtx = value

    @property
    def dist_coeff(self):
        return self.__dist_coeff
    @dist_coeff.setter
    def dist_coeff(self, value):
        self.__dist_coeff = value

    @property
    def rvecs(self):
        return self.__rvecs
    @rvecs.setter
    def rvecs(self, value):
        self.__rvecs = value

    @property
    def tvecs(self):
        return self.__tvecs
    @tvecs.setter
    def tvecs(self, value):
        self.__tvecs = value


    def __calibrate_chessboard(self, fnames, pattern_size, criteria):
        obj_points = np.zeros((pattern_size[0]*pattern_size[1], 3), np.float32)
        obj_points[:,:2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape((-1, 2))

        object_points = []
        image_points = []
        for fname in fnames:
            img = mpimg.imread(fname)
            img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            found, img_points = cv2.findChessboardCorners(img_gray, pattern_size, None)

            if found:
                cv2.cornerSubPix(img_gray,img_points,(11,11),(-1,-1),criteria)
                object_points.append(obj_points)
                image_points.append(img_points)

        ret, mtx, dst, rvec, tvec =  cv2.calibrateCamera(object_points, image_points, img.shape[1::-1], None, None)
        self.__ret = ret
        self.__cam_mtx = mtx
        self.__dist_coeff = dst
        self.__rvecs = rvec
        self.__tvecs = tvec
