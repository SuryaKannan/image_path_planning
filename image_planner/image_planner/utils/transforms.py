#!/usr/bin/python3 
import numpy as np

def transform_points(T, trajectories):
    # apply transformation to a set of coordinates using homogenous transform matrix and a set of 3D trajectories
    return np.linalg.inv(T)@trajectories

def project_points(K, points):
    # project a set of 3D world points back into the image space 
    im_coords_scaled = K@points[:-1,:]
    im_coords = im_coords_scaled/im_coords_scaled[-1,:]
    im_coords = im_coords.astype(np.int32)

    return im_coords