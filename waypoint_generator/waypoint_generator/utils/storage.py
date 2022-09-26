#!/usr/bin/python3 
import os 
import numpy as np

def update_param(path,filename,data):
    if os.path.isdir(path):
        if filename == "waypoints.npy":
            data = np.reshape(data,(data.shape[0]*data.shape[1],data.shape[2]))
        np.save(os.path.join(path,filename),data)

    return

