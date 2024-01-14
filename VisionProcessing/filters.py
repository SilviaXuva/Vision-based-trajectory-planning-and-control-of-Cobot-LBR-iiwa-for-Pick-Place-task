from VisionProcessing.guiFeatures import drawingCircle, pink
from VisionProcessing.preProcessing import data

import cv2
import numpy as np

data = {
    "blur_ksize": (5,5),
    "blur_sigmax": 0,
    "canny_a": 10,
    "canny_b": 50,
    "thresh_thresh": 0,
    "thresh_maxval": 255,
    "corners_gf_maxcorners": 100,
    "corners_gf_qualitylevel": 0.01,
    "corners_gf_mindistance": 10,
    "corners_h_blocksize": 2,
    "corners_h_ksize": 3,
    "corners_h_k": 0.04,
}

# ============== Blur ===================
def GetBlur(img: np.ndarray, ksize: tuple = data["blur_ksize"], sigmax: int = data["blur_sigmax"]):
    blur = cv2.GaussianBlur(img, ksize = ksize, sigmaX = sigmax)
    return blur

# ============== Canny ===================
def GetCanny(img: np.ndarray, threshold1: int = data["canny_a"], threshold2: int = data["canny_b"]):
    canny = cv2.Canny(img, threshold1 = threshold1, threshold2 = threshold2)
    return canny
    
# ============== Threshold ===================
def GetThreshold(img: np.ndarray, thresh: int = data["thresh_thresh"], maxval: int = data["thresh_maxval"], type: int = cv2.THRESH_BINARY):
    ret, threshold = cv2.threshold(img, thresh = thresh, maxval = maxval, type = type)
    return threshold

# ============== Contours ===================
def GetContours(img: np.ndarray, mode: int = 1, method: int = 2):
    contours = cv2.findContours(img, mode = mode, method = method)
    cnts = contours[0] if len(contours) == 2 else contours[1]
    return cnts

# ============== Corners By Good Features ===================
def GetCornersByGoodFeatures(img: np.ndarray, filtered: np.ndarray, draw: bool = False, max_corners: int = data["corners_gf_maxcorners"], quality_level: float = data["corners_gf_qualitylevel"], min_distance: float = data["corners_gf_mindistance"]):
    corners = cv2.goodFeaturesToTrack(filtered, maxCorners = max_corners, qualityLevel = quality_level, minDistance = min_distance)
    if draw:
        img = img.copy()
        try:
            corners = np.intp(corners)
            for corner in corners:
                x,y = corner.ravel()
                drawingCircle(img, (x,y))
                drawingCircle(img, (x,y))
        except:
            print('Good features to track returned blank')
    return corners, img

# ============== Corners By Harris ===================
def GetCornersByHarris(img: np.ndarray, filtered: np.ndarray, draw: bool = False, block_size: int = data["corners_h_blocksize"], ksize: int = data["corners_h_ksize"], k: float = data["corners_h_k"]):
    dst = cv2.cornerHarris(np.float32(filtered), blockSize = block_size, ksize = ksize, k = k)
    if draw:
        img[dst > 0.01 * dst.max()] = pink
    return dst, img

# ============== Refine Corners ===================
def RefineCorners(img: np.ndarray, corners: np.ndarray, win_size: tuple(int) = (11, 11), zero_zone: tuple(int) = (-1, -1), criteria: tuple(int, float) = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)):
    corners = cv2.cornerSubPix(img, corners, winSize = win_size, zeroZone = zero_zone, criteria = criteria)
    return corners

# ============== Not working ===================
# def getAdaptativeThreshold(img):
#     return cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,11,2)
# ==============================================