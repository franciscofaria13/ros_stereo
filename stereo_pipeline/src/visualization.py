import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField

def publish_pointcloud(points, frame_id="camera_link"):
    header = rospy.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    # Convert 3D points into PointCloud2 format
    cloud_msg = pc2.create_cloud_xyz32(header, points)

    return cloud_msg