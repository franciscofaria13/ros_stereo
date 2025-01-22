#NO MORE MORPHOLOGICAL DIFFERENCES FROM RECTIFIED IMAGE
""" import cv2

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

    return disparity """


import cv2

def compute_disparity(left_img, right_img):
    # Ensure images are in grayscale
    if len(left_img.shape) == 3:
        left_img = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
        right_img = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

    left_edges = cv2.Canny(left_img, 220, 270)
    right_edges = cv2.Canny(right_img, 220, 270)

    # Stereo Matching using SGBM
    stereo = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=128,  # Adjust based on scene
        blockSize=3,        # Smaller block size for narrow objects
        P1=8 * 3 * 5**2,
        P2=32 * 3 * 5**2,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=50,
        speckleRange=1
    )

    # Compute disparity
    
    disparity_map = stereo.compute(left_edges, right_edges)
    disparity = cv2.normalize(disparity_map, None, 0, 255, cv2.NORM_MINMAX)

    return disparity


"""     stereo = cv2.StereoSGBM_create(
    minDisparity=0,  # Minimum possible disparity value
    numDisparities=16*6,  # Must be divisible by 16, higher values improve depth range
    blockSize=9,  # Larger values for smoother disparity
    P1=8*3*9**2,  # Controls the smoothness of disparity (small P1 for fine details)
    P2=32*3*9**2,  # Larger P2 to encourage smoother disparity map
    disp12MaxDiff=1,  # Maximum allowed difference in left-right disparity check
    uniquenessRatio=10,  # Filters low-quality disparities (increase to remove noise)
    speckleWindowSize=100,  # Reduces speckle noise (increase to remove noise)
    speckleRange=32,  # Disparity range for speckle filtering
    preFilterCap=63,  # Clipping values of pixel intensities
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY  # Default mode, change if needed
) """