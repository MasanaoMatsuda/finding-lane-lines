from enum import Enum
import cv2

class PerspectiveType(Enum):
    Normal = 1
    BirdsEye = 2

class Transform:
    def __init__(self, image, src, dst):
        self.vertical = image.shape[0]
        self.horizontal = image.shape[1]
        self.M = cv2.getPerspectiveTransform(src, dst)
        self.Minv = cv2.getPerspectiveTransform(dst, src)

    def perspective(self, image, ptype, flags=cv2.INTER_LINEAR):
        if ptype == PerspectiveType.Normal:
            mat = self.Minv
        else:
            mat = self.M
        return cv2.warpPerspective(image, mat, (self.horizontal, self.vertical), flags=flags)
