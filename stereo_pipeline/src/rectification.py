import cv2
import numpy as np

def rectify_images(left_img, right_img, K_left, D_left, K_right, D_right, R, T):
    h, w = left_img.shape[:2]

    K_left = np.array(K_left, dtype=np.float64).reshape(3,3)
    D_left = np.array(D_left, dtype=np.float64)
    K_right = np.array(K_right, dtype=np.float64).reshape(3,3)
    D_right = np.array(D_right, dtype=np.float64)
    R = np.array(R, dtype=np.float64).reshape(3, 3)
    T = np.array(T, dtype=np.float64)

    # Perform stereo rectification using OpenCV
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        K_left, D_left, 
        K_right, D_right, 
        (w, h), 
        R, T, 
        flags=cv2.CALIB_ZERO_DISPARITY, 
        alpha=0  # Set to 0 to crop out black areas after rectification
    )

    # Generate rectification maps
    mapx1, mapy1 = cv2.initUndistortRectifyMap(K_left, D_left, R1, P1, (w, h), cv2.CV_32FC1)
    mapx2, mapy2 = cv2.initUndistortRectifyMap(K_right, D_right, R2, P2, (w, h), cv2.CV_32FC1)

    # Rectify images
    left_rect = cv2.remap(left_img, mapx1, mapy1, cv2.INTER_LINEAR)
    right_rect = cv2.remap(right_img, mapx2, mapy2, cv2.INTER_LINEAR)

    return left_rect, right_rect, Q

