import numpy as np
import numpy.typing as npt
from geometry_msgs.msg import Vector3, Quaternion, Point
from typing import Any, Union


def _translation_to_numpy(translation: Vector3) -> npt.NDArray[np.float64]:
    return np.array([translation.x, translation.y, translation.z])


def _point_to_numpy(point: Point) -> npt.NDArray[np.float64]:
    return np.array([point.x, point.y, point.z])


def _rotation_to_numpy(rotation: Quaternion) -> npt.NDArray[np.float64]:
    return np.array([rotation.x, rotation.y, rotation.z, rotation.w])


def normalized(v: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
    norm: float = np.linalg.norm(v)  # type: ignore
    return v / norm


def perpendicular_2d(v: npt.NDArray[Any]) -> npt.NDArray[Any]:
    if v.shape != (2,) and v.shape != (1, 2) and v.shape != (2, 1):
        raise Exception("Vector must be 2D")
    orig_shape = v.shape
    return np.reshape((np.array([-v.flatten()[1], v.flatten()[0]])), orig_shape)


def rotate_2d(v: npt.NDArray[Any], angle: float) -> npt.NDArray[Any]:
    """
    rotates a 2D vector by angle radians
    :param v: the vector to rotate
    :param angle: the angle to rotate by
    :return: the rotated vector
    """
    if v.shape != (2,) and v.shape != (1, 2) and v.shape != (2, 1):
        raise Exception("Vector must be 2D")
    orig_shape = v.shape
    v = v.flatten()
    rotation_mat = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    return np.reshape(np.matmul(rotation_mat, v), orig_shape)


def angle_to_rotate(v1: npt.NDArray[np.float64], v2: npt.NDArray[np.float64]) -> float:
    """
    returns the angle you'd need to rotate normalized(v1) by in order to have a vector that is normalized(v2)
    """
    v1_u, v2_u = normalized(v1), normalized(v2)
    smallest_angle: float = np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
    # Determine the sign of our effort by seeing if we are to the left or to the right of the target
    # This is done by dotting rover_dir and target_dir rotated 90 degrees ccw
    perp_alignment: float = v2_u[0] * -v1_u[1] + v2_u[1] * v1_u[0]
    sign: float = np.sign(perp_alignment)
    if sign == 0.0:
        sign = 1
    return smallest_angle * sign


def numpify(msg: Union[Vector3, Quaternion, Point]) -> npt.NDArray[Any]:
    match msg:
        case Vector3():  # type: ignore
            return _translation_to_numpy(msg)
        case Quaternion():  # type: ignore
            return _rotation_to_numpy(msg)
        case Point():  # type: ignore
            return _point_to_numpy(msg)
    raise Exception("Message must be a Vector3, Quaternion, or Point")
