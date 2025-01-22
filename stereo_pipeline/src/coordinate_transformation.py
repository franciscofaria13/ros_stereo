import numpy as np
import tf

def transform_to_world(points_3D, pose):
    translation = np.array([pose.position.x, pose.position.y, pose.position.z])
    rotation_matrix = tf.transformations.quaternion_matrix([
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    ])[:3, :3]

    transformed_points = np.dot(rotation_matrix, points_3D.T).T + translation

    return transformed_points


