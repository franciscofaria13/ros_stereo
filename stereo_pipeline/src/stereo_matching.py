import cv2

def compute_disparity(left_img, right_img):
    # Ensure images are in grayscale
    if len(left_img.shape) == 3:
        left_img = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
        right_img = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

    # StereoSGBM parameters (adjust based on requirements)
    stereo = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=16 * 6,  # Must be a multiple of 16
        blockSize=9,
        P1=8 * 3 * 9**2,
        P2=32 * 3 * 9**2,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32
    )

    # Compute disparity
    disparity = stereo.compute(left_img, right_img).astype('float32') / 16.0

    return disparity
