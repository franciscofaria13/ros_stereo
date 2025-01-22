#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

class TFBroadcaster:
    def __init__(self):
        rospy.init_node('tf_broadcaster', anonymous=True)

        # Initialize the TF broadcaster
        self.br = tf.TransformBroadcaster()

        # Subscribe to the localization data
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_position_callback)

        # Set up periodic updates
        self.rate = rospy.Rate(10)

    def local_position_callback(self, msg):
        # Extract position and orientation from the PoseStamped message
        position = msg.pose.position
        orientation = msg.pose.orientation

        # Broadcast the transform
        self.br.sendTransform(
            (position.x, position.y, position.z),  # Translation
            (orientation.x, orientation.y, orientation.z, orientation.w),  # Rotation (Quaternion)
            rospy.Time.now(),
            "camera_link",  # Child frame
            "map"  # Parent frame (could be "world" or "odom")
        )

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        broadcaster = TFBroadcaster()
        broadcaster.run()
    except rospy.ROSInterruptException:
        pass
