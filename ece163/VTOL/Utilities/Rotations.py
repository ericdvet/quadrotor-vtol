"""
Author: Eric Vetha (evetha@ucsc.edu)
This file contains functions for converting between Euler angles and Direction Cosine Matrices (DCMs).
"""

import math

def euler2DCM(yaw, pitch, roll):
    """
    Converts Euler angles to a Direction Cosine Matrix (DCM)
    
    :param yaw: rotation about z-axis [rad]
    :param pitch: rotation about y-axis [rad]
    :param roll: rotation about x-axis [rad]
    :return: Direction Cosine Matrix (list of lists) [3 x 3]
    """
    R = [
        [math.cos(yaw)*math.cos(pitch), math.sin(yaw)*math.cos(pitch), -math.sin(pitch)],
        [math.cos(yaw)*math.sin(pitch)*math.sin(roll) - math.sin(yaw)*math.cos(roll), math.sin(yaw)*math.sin(pitch)*math.sin(roll) + math.cos(yaw)*math.cos(roll), math.cos(pitch)*math.sin(roll)],
        [math.cos(yaw)*math.sin(pitch)*math.cos(roll) + math.sin(yaw)*math.sin(roll), math.sin(yaw)*math.sin(pitch)*math.cos(roll) - math.cos(yaw)*math.sin(roll), math.cos(pitch)*math.cos(roll)]
    ]
    return R

def dcm2Euler(dcm):
    """
    Converts a Direction Cosine Matrix (DCM) to Euler angles
    
    :param dcm: Direction Cosine Matrix (list of lists) [3 x 3]
    :return: yaw, pitch, roll [rad]
    """
    if (dcm[0][2] < 1 and dcm[0][2] > -1):
        pitch = -math.asin(dcm[0][2])
    elif dcm[0][2] <= -1:
        pitch = math.pi/2
    else:
        pitch = -math.pi/2
    roll = math.atan2(dcm[1][2], dcm[2][2])
    yaw = math.atan2(dcm[0][1], dcm[0][0])
    return yaw, pitch, roll

def ned2enu(points):
    """
    Converts points from NED to ENU frame
    
    :param points: points in NED frame (list of lists) [n x 3]
    :return: points in ENU frame (list of lists) [n x 3]
    """
    temp = [[0] * 3 for i in range(len(points))]
    for i in range(len(points)):
        temp[i][0] = points[i][1]
        temp[i][1] = points[i][0]
        temp[i][2] = -1 * points[i][2]
    return temp