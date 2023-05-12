#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 21 17:27:37 2023

@author: shreyash
"""

import numpy as np 
import matplotlib.pyplot as plt 
import os 
import wall_localization




lidar_data = np.load('lidar_scan.npy',allow_pickle=True)
lidar_data = lidar_data[0,3:]
print(len(lidar_data))


robotPos = [-1,0,0]
robotX = robotPos[0]
robotY = robotPos[1]
robotTheta = robotPos[2]

unrobotX, unrobotY, unrobotTheta = robotPos

SL, SR = 0,0
P = np.eye(3)

step_size = np.pi/1147 * 2
odometry_data = [0,SL, SR]
[robotX, robotY, robotTheta, unrobotX, unrobotY, unrobotTheta, P] = wall_localization.localize(lidar_data, step_size, odometry_data, robotX, robotY, robotTheta, unrobotX, unrobotY, unrobotTheta, P)

print(robotX, robotY, robotTheta)


