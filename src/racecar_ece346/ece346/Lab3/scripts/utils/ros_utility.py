from geometry_msgs.msg import Pose
import numpy as np
from scipy.spatial.transform import Rotation as R

def pose2T(pose: Pose):
    '''
    Convert a geometry_msgs/Pose message to a SE(3) matrix
    Args:
        pose: geometry_msgs/Pose message
    Return:
        T: a 4x4 numpy array
    '''
    orientation = pose.orientation
    position = pose.position
    quat = [orientation.x, orientation.y, orientation.z, orientation.w]
    trans = [position.x, position.y, position.z]
    T = np.eye(4)
    T[:3, :3] = R.from_quat(quat).as_matrix()
    T[:3, 3] = trans
    return T