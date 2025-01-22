#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge
from std_msgs.msg import Header
import cv2
from line_detection import detect_lines
from stereo_matching import compute_disparity
from pointcloud_processing import disparity_to_3D
from coordinate_transformation import transform_to_world
from visualization import publish_pointcloud
from rectification import rectify_images 
import numpy as np

class StereoProcessor:
    def __init__(self):
        rospy.init_node('stereo_processor', anonymous=True)

        # Subscribe to stereo images and calibration data
        self.left_image_sub = rospy.Subscriber("/stereo/left/image_raw", Image, self.left_image_callback)
        self.right_image_sub = rospy.Subscriber("/stereo/right/image_raw", Image, self.right_image_callback)
        self.left_info_sub = rospy.Subscriber("/stereo/left/camera_info", CameraInfo, self.left_info_callback)
        self.right_info_sub = rospy.Subscriber("/stereo/right/camera_info", CameraInfo, self.right_info_callback)

        # Publishers for intermediate outputs
        self.rectified_left_pub = rospy.Publisher("/debug/rectified_left", Image, queue_size=1)
        self.rectified_right_pub = rospy.Publisher("/debug/rectified_right", Image, queue_size=1)
        self.detected_lines_pub = rospy.Publisher("/debug/detected_lines", Image, queue_size=1)
        self.disparity_pub = rospy.Publisher("/debug/disparity", Image, queue_size=1)
        self.pointcloud_pub = rospy.Publisher("/debug/pointcloud", PointCloud2, queue_size=1)

        self.bridge = CvBridge()
        self.left_image = None
        self.right_image = None
        self.K_left = None
        self.D_left = None
        self.K_right = None
        self.D_right = None

        # Timer to periodically call the process function
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def timer_callback(self, event):
        self.process()

    def left_image_callback(self, msg):
        self.left_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")   #mono8 for GREY and bgr8 for color

    def right_image_callback(self, msg):
        self.right_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def left_info_callback(self, msg):
        """Callback function to handle left camera calibration data."""
        self.K_left = np.array(msg.K, dtype=np.float64).reshape(3, 3)
        self.D_left = np.array(msg.D, dtype=np.float64)
        self.R_left = np.array(msg.R, dtype=np.float64).reshape(3, 3)
        self.P_left = np.array(msg.P, dtype=np.float64).reshape(3, 4)

        rospy.loginfo("Received left camera info.")
        rospy.loginfo("K_left:\n{}".format(self.K_left))
        rospy.loginfo("D_left: {}".format(self.D_left))
        rospy.loginfo("R_left:\n{}".format(self.R_left))
        rospy.loginfo("P_left:\n{}".format(self.P_left))

    def right_info_callback(self, msg):
        """Callback function to handle right camera calibration data."""
        self.K_right = np.array(msg.K, dtype=np.float64).reshape(3, 3)
        self.D_right = np.array(msg.D, dtype=np.float64)
        self.R_right = np.array(msg.R, dtype=np.float64).reshape(3, 3)
        self.P_right = np.array(msg.P, dtype=np.float64).reshape(3, 4)

        rospy.loginfo("Received right camera info.")
        rospy.loginfo("K_right:\n{}".format(self.K_right))
        rospy.loginfo("D_right: {}".format(self.D_right))
        rospy.loginfo("R_right:\n{}".format(self.R_right))
        rospy.loginfo("P_right:\n{}".format(self.P_right))

    def publish_image(self, pub, cv_image):
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        pub.publish(ros_image)

    def send_pointcloud(self, points):
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "camera_link"
        
        # Call the function from visualization.py correctly
        cloud_msg = publish_pointcloud(points, frame_id="camera_link")
        
        self.pointcloud_pub.publish(cloud_msg)


    # def process(self):
    #     if self.left_image is not None and self.right_image is not None and self.K_left is not None and self.K_right is not None:
    #         rospy.loginfo("Processing stereo images")

    #         # Step 1: Rectify Images
    #         left_rectified, right_rectified, _ = rectify_images(
    #             self.left_image, self.right_image, 
    #             np.array(self.K_left).reshape(3, 3), np.array(self.D_left),
    #             np.array(self.K_right).reshape(3, 3), np.array(self.D_right),
    #             np.array(self.R_left).reshape(3, 3), np.array(self.P_right)[:, 3] / self.P_right[0, 0]
    #         )

    #         self.publish_image(self.rectified_left_pub, left_rectified)
    #         self.publish_image(self.rectified_right_pub, right_rectified)

    #         # Step 2: Line Detection
    #         detected_lines_img, detected_rois = detect_lines(left_rectified)
    #         self.publish_image(self.detected_lines_pub, detected_lines_img)

    #         if detected_rois:
    #             rospy.loginfo("Detected ROIs: {}".format(detected_rois))

    #             # Select the best ROI based on area
    #             best_roi = max(detected_rois, key=lambda roi: (roi[2] - roi[0]) * (roi[3] - roi[1]))
    #             rospy.loginfo("Best ROI selected: {}".format(best_roi))

    #             # Expand the ROI to ensure it's sufficiently large
    #             min_width = 300  # Set minimum width in pixels
    #             min_height = 300  # Set minimum height in pixels

    #             x1, y1, x2, y2 = best_roi

    #             # Expand the region if too small
    #             if (x2 - x1) < min_width:
    #                 x2 = x1 + min_width
    #             if (y2 - y1) < min_height:
    #                 y2 = y1 + min_height

    #             # Ensure the ROI stays within the image bounds
    #             x1 = max(0, x1)
    #             y1 = max(0, y1)
    #             x2 = min(left_rectified.shape[1], x2)
    #             y2 = min(left_rectified.shape[0], y2)

    #             best_roi = (x1, y1, x2, y2)
    #             rospy.loginfo("Expanded ROI: {}".format(best_roi))

    #             disparity = compute_disparity(left_rectified, right_rectified, best_roi)
    #             disparity_color = cv2.applyColorMap(cv2.convertScaleAbs(disparity, alpha=0.5), cv2.COLORMAP_JET)
    #             self.publish_image(self.disparity_pub, disparity_color)
    #         else:
    #             rospy.logwarn("No valid ROIs detected.")

    def process(self):
        if self.left_image is not None and self.right_image is not None and self.K_left is not None and self.K_right is not None:
            rospy.loginfo("Processing stereo images")

            # Step 1: Rectify Images
            left_rectified, right_rectified, Q = rectify_images(
                self.left_image, self.right_image, 
                np.array(self.K_left).reshape(3, 3), np.array(self.D_left),
                np.array(self.K_right).reshape(3, 3), np.array(self.D_right),
                np.array(self.R_left).reshape(3, 3), np.array(self.P_right)[:, 3] / self.P_right[0, 0]
            )

            # Publish rectified images for debugging
            self.publish_image(self.rectified_left_pub, left_rectified)
            self.publish_image(self.rectified_right_pub, right_rectified)

            # Step 2: Line Detection (for visualization only)
            detected_lines_img, _ = detect_lines(left_rectified)
            self.publish_image(self.detected_lines_pub, detected_lines_img)

            # Step 3: Stereo Matching (without ROI filtering)
            rospy.loginfo("Computing disparity on full image")
            disparity = compute_disparity(left_rectified, right_rectified)

            # Convert disparity to color map and publish
            disparity_color = cv2.applyColorMap(cv2.convertScaleAbs(disparity, alpha=0.5), cv2.COLORMAP_JET)
            self.publish_image(self.disparity_pub, disparity_color)

            # Step 4: 3D Reconstruction
            points_3D = disparity_to_3D(disparity, Q)
            self.send_pointcloud(points_3D)


            
    def run(self):
        rate = rospy.Rate(10)  # Process at 10 Hz
        while not rospy.is_shutdown():
            self.process()
            rate.sleep()



if __name__ == "__main__":
    #rospy.init_node('stereo_processor', anonymous=False)
    processor = StereoProcessor()
    rospy.spin(ros.ROSInterruptException, rospy.MultiThreadedSpinner(4))


