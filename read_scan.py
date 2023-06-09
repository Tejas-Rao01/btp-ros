#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Feb  3 19:18:08 2023

@author: shreyash
"""

import yaml
import os
import pickle
import numpy as np

stream = open("/home/shreyash/Desktop/btp/ros/debug_scan.yaml", "r")

docs = yaml.load_all(stream, yaml.FullLoader)

lidar_scan = np.zeros(shape=(45, 1150))

for i, doc in enumerate(docs):
    #print(doc.keys())
    print(i)
    if i >= 10:
        break

    min_angle = doc['angle_min']
    max_angle = doc['angle_max']
    angle_increment = doc['angle_increment']
    lidar_data = doc['ranges']
    
    lidar_scan[i, 0] =  0
    lidar_scan[i, 1] = max_angle * 2
    lidar_scan[i, 2] = angle_increment
    lidar_scan[i, 3:] = lidar_data
    
print(lidar_scan)
np.save('lidar_scan.npy', lidar_scan)
print(np.max(lidar_scan))
print(max_angle)
    
    