import math
import numpy as np
import cv2


def aruco_poses_to_transforms(
        detected_ids,
        corners,
        rvecs,
        only_z_rot=True):
    """
    Calculates rotation matrix to euler angles
    The result is the same as MATLAB except the order
    of the euler angles ( x and z are swapped ).
    https://www.learnopencv.com/rotation-matrix-to-euler-angles/

    Args:
        detected_ids ([int]): Detected aruco marker ids
        corners (?): List of Detected aruco marker corner's
                    from aruco.detectMarkers
        rvecs (?): ?

    Returns : dictionary
        key : aruco_id : int
        value:
            position : numpy array[x, y]
                x and y positions are in pixel coordinates from
                zero to image height/width in pixels.
            rotation : numpy array[rot_x, rot_y, rot_z] or
                        numpy array[rot_z]
                Rotations are from -180 to 180 degrees
    """
    robot_trans_dict = {}

    if detected_ids is None or corners is None or rvecs is None:
        return robot_trans_dict

    for id, corner_list, rvec in zip(detected_ids, corners, rvecs):
        robot_trans_dict[id.item()] = {}
        center = np.mean(corner_list[0], axis=0, dtype=np.float32)
        dst, jacobian = cv2.Rodrigues(rvec[0])
        euler_angles = _rotation_matrix_to_euler_angles(dst)
        robot_trans_dict[id.item()]['position'] = center
        if only_z_rot:
            robot_trans_dict[id.item()]['rotation'] = \
                np.array([euler_angles[2]])
        else:
            robot_trans_dict[id.item()]['rotation'] = euler_angles

    return robot_trans_dict

def _is_rotation_matrix(matrix):
    """
    Checks if a matrix is a valid rotation matrix.
    https://www.learnopencv.com/rotation-matrix-to-euler-angles/

    Args:
        matrix (?): OpenCV matrix

    Returns:
        boolean: Is the input matrix rotation matrix
    """
    r_t = np.transpose(matrix)
    should_be_identity = np.dot(r_t, matrix)
    identity = np.identity(3, dtype=matrix.dtype)
    n = np.linalg.norm(identity - should_be_identity)
    return n < 1e-6

def _rotation_matrix_to_euler_angles(rotation_matrix):
    """
    Calculates rotation matrix to euler angles
    The result is the same as MATLAB except the order
    of the euler angles ( x and z are swapped ).
    https://www.learnopencv.com/rotation-matrix-to-euler-angles/

    Args:
        rotation_matrix (int): Rotation matrix to transform

    Returns:
        numpy array [3]: x, y and z euler rotations
    """
    assert _is_rotation_matrix(rotation_matrix)
    sy = math.sqrt(
        rotation_matrix[0, 0] * rotation_matrix[0, 0] +
        rotation_matrix[1, 0] * rotation_matrix[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        y = math.atan2(-rotation_matrix[2, 0], sy)
        z = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    else:
        x = math.atan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
        y = math.atan2(-rotation_matrix[2, 0], sy)
        z = 0

    return np.array([
            math.degrees(x),
            math.degrees(y),
            math.degrees(z)],
        dtype=np.float32)
