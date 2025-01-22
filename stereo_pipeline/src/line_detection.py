import cv2
import numpy as np

def detect_lines(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 220, 270, apertureSize=3)

    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, 270, maxLineGap=10)   #1, np.pi/180, 270, None, 0, 0
    roi = []

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            roi.append((x1, y1, x2, y2))

    return image, roi

