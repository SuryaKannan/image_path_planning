#!/usr/bin/python3 
import numpy as np
import math

def transform_points(T, trajectories):
    # apply transformation to a set of coordinates using homogenous transform matrix and a set of 3D trajectories
    return np.linalg.inv(T)@trajectories

def project_points(K, points):
    # project a set of 3D world points back into the image space 
    im_coords_scaled = K@points[:-1,:]
    im_coords = im_coords_scaled/im_coords_scaled[-1,:]
    im_coords = im_coords.astype(np.int32)

    return im_coords

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians