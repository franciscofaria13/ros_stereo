import cv2
import numpy as np

def disparity_to_3D(disparity, Q):
    points_3D = cv2.reprojectImageTo3D(disparity, Q)
    mask = disparity > 0  # Remove pixels sem disparidade
    filtered_points = points_3D[mask]

    return filtered_points

