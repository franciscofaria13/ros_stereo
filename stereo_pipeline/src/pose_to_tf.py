#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PoseStamped

class TFBroadcaster:
    def __init__(self):
        rospy.init_node('tf_broadcaster', anonymous=True)

        # Initialize the TF broadcaster to publish transformations
        self.br = tf.TransformBroadcaster()

        # Get parent and child frame names from ROS parameters, with default values
        self.parent_frame = rospy.get_param("~parent_frame", "map")  # Default parent frame: 'map'
        self.child_frame = rospy.get_param("~child_frame", "camera_link")  # Default child frame: 'camera_link'

        # Subscribe to the local position topic to receive pose updates
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)

        rospy.loginfo("TF broadcaster initialized: {} -> {}".format(self.parent_frame, self.child_frame))

    def pose_callback(self, msg):
        # Extract the position and orientation from the PoseStamped message
        position = msg.pose.position
        orientation = msg.pose.orientation

        rospy.loginfo("Received pose: x={}, y={}, z={}".format(position.x, position.y, position.z))

        # Publish a dynamic transformation between the parent and child frames
        self.br.sendTransform(
            (position.x, position.y, position.z),  # Translation (x, y, z)
            (orientation.x, orientation.y, orientation.z, orientation.w),  # Rotation (quaternion)
            rospy.Time.now(),
            self.child_frame,  # The child frame (camera_link)
            self.parent_frame  # The parent frame (map)
        )

        # Publish a static transformation for an additional unknown frame relationship
        self.br.sendTransform(
            (0.0, 0.0, 0.0),  # No translation (placeholder)
            (0.0, 0.0, 0.0, 1.0),  # Identity quaternion (no rotation)
            rospy.Time.now(),
            "base_link",  # Child frame (static)
            self.child_frame  # Parent frame (camera_link)
        )

    def run(self):
        # Keep the node running to continuously broadcast transformations
        rospy.spin()

if __name__ == "__main__":
    try:
        broadcaster = TFBroadcaster()
        broadcaster.run()
    except rospy.ROSInterruptException:
        pass  # Gracefully exit if the node is interrupted
