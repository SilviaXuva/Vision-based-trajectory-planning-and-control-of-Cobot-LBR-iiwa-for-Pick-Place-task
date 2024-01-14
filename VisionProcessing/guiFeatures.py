import cv2
import numpy as np

blue = (255, 0, 0)
green = (0, 255, 0)
red = (0, 0, 255)
white = (255, 255, 255)
yellow = (255, 255, 0)
pink = (255, 100, 255)
black = (0, 0, 0)

# Write Text
def WriteText(img: np.ndarray, text, origin, fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale = 1, color = pink, thickness = 1):
    cv2.putText(img, str(text), origin, fontFace, fontScale, color, thickness)

# Drawing Circle
def DrawingCircle(img: np.ndarray, center, radius = 3, color = pink, thickness = -1):
    cv2.circle(img, center, radius, color, thickness)

# Drawing Rectangle
def DrawingRectangle(img: np.ndarray, pt1, pt2, color = pink, thickness = 2):
    cv2.rectangle(img, pt1, pt2, color, thickness)

# Drawing Countours
def DrawingContours(img: np.ndarray, contours, contourIdx = -1, color = pink, thickness = 3):
    cv2.drawContours(img, contours, contourIdx, color, thickness)

# Drawing Frame
def DrawingFrame(img: np.ndarray, mtx, dist_coeff, rvec, tvec, length):
    cv2.drawFrameAxes(img, mtx, dist_coeff, rvec, tvec, length)