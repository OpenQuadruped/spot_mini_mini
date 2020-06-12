#!/usr/bin/env python
import numpy as np

# NOTE: Code snippets from Modern Robotics at Northwestern University:
# See https://github.com/NxRLab/ModernRobotics


def RpToTrans(R, p):
    """Converts a rotation matrix and a position vector into homogeneous
    transformation matrix

    :param R: A 3x3 rotation matrix
    :param p: A 3-vector
    :return: A homogeneous transformation matrix corresponding to the inputs

    Example Input:
        R = np.array([[1, 0,  0],
                      [0, 0, -1],
                      [0, 1,  0]])
        p = np.array([1, 2, 5])
    Output:
        np.array([[1, 0,  0, 1],
                  [0, 0, -1, 2],
                  [0, 1,  0, 5],
                  [0, 0,  0, 1]])
    """
    return np.r_[np.c_[R, p], [[0, 0, 0, 1]]]


def TransToRp(T):
    """Converts a homogeneous transformation matrix into a rotation matrix
    and position vector

    :param T: A homogeneous transformation matrix
    :return R: The corresponding rotation matrix,
    :return p: The corresponding position vector.

    Example Input:
        T = np.array([[1, 0,  0, 0],
                      [0, 0, -1, 0],
                      [0, 1,  0, 3],
                      [0, 0,  0, 1]])
    Output:
        (np.array([[1, 0,  0],
                   [0, 0, -1],
                   [0, 1,  0]]),
         np.array([0, 0, 3]))
    """
    T = np.array(T)
    return T[0: 3, 0: 3], T[0: 3, 3]


def TransInv(T):
    """Inverts a homogeneous transformation matrix

    :param T: A homogeneous transformation matrix
    :return: The inverse of T
    Uses the structure of transformation matrices to avoid taking a matrix
    inverse, for efficiency.

    Example input:
        T = np.array([[1, 0,  0, 0],
                      [0, 0, -1, 0],
                      [0, 1,  0, 3],
                      [0, 0,  0, 1]])
    Output:
        np.array([[1,  0, 0,  0],
                  [0,  0, 1, -3],
                  [0, -1, 0,  0],
                  [0,  0, 0,  1]])
    """
    R, p = TransToRp(T)
    Rt = np.array(R).T
    return np.r_[np.c_[Rt, -np.dot(Rt, p)], [[0, 0, 0, 1]]]


def RPY(roll, pitch, yaw):
    Roll = np.array([[1, 0, 0, 0],
                    [0, np.cos(roll), -np.sin(roll), 0],
                    [0, np.sin(roll), np.cos(roll), 0],
                    [0, 0, 0, 1]])
    Pitch = np.array([[np.cos(pitch), 0, np.sin(pitch), 0],
                     [0, 1, 0, 0],
                     [-np.sin(pitch), 0, np.cos(pitch), 0],
                     [0, 0, 0, 1]])
    Yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0, 0],
                    [np.sin(yaw), np.cos(yaw), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    return np.matmul(np.matmul(Roll, Pitch), Yaw)


def RotateTranslate(rotation, position):
    trans = np.eye(4)
    trans[0, 3] = position[0]
    trans[1, 3] = position[1]
    trans[2, 3] = position[2]

    return np.dot(rotation, trans)


def TransformVector(xyz_coord, rotation, translation):
    xyz_vec = np.append(xyz_coord, 1.0)

    Transformed = np.dot(RotateTranslate(rotation, translation), xyz_vec)
    return Transformed[:3]
