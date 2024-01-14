from VisionProcessing.preProcessing import data

import cv2
import numpy as np

ranges = {
    "red": {
        "low": data['red_hsv_low'],
        "high": data['red_hsv_high']
    },
    "green": {
        "low": data['green_hsv_low'],
        "high": data['green_hsv_high']
    },
    "blue": {
        "low": data['blue_hsv_low'],
        "high": data['blue_hsv_high']
    }
}
# Color extraction based on HSV
def MaskRanges(img: np.ndarray, ranges: dict = ranges):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    whole_mask = None
    colors = list()
    for key in ranges.keys():
        mask = cv2.inRange(hsv, ranges[key]['low'], ranges[key]['high'])
        if whole_mask is not None:
            whole_mask += mask
        else:
            whole_mask = mask
        if np.any(mask[:, :] == 255):
            colors.append(key)
    cropped = img.copy()
    cropped[np.where(whole_mask == 255)] = 255
    return cropped, colors

def RemoveBackground(img: np.ndarray, hsv_low: np.ndarray = data['floor_hsv_low'], hsv_high: np.ndarray = data['floor_hsv_high']):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, hsv_low, hsv_high)
    cropped = img - cv2.bitwise_and(img, img, mask=mask)
    return cropped

def GetGray(img: np.ndarray, color: int = cv2.COLOR_RGB2GRAY):
    return cv2.cvtColor(img, color)