#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 23 10:18:48 2023
@author: shreyash
"""



import robot_params 
import math
import localization_constants
import matplotlib.pyplot as plt
import wall_ekf
import numpy as np

def localize(lidar_data, step_size, odometry_data, robotX, robotY, robotTheta,unrobotX, unrobotY, unrobotTheta, P):
    
    
    lidar_data = process_lidar_data(lidar_data, step_size)
    #plt.scatter(robotX, robotY,c='red', marker='x')
    # Check 
    [odomTime, SL, SR] = odometry_data
        
    [unrobotX, unrobotY, unrobotTheta] = get_pred_pos(SL, SR, unrobotX, unrobotY, unrobotTheta)    

    robotX, robotY, robotTheta, P, walls = wall_ekf.kalman_filter([robotX, robotY, robotTheta], lidar_data, P, SL, SR)
    #plot_walls(walls, robotX, robotY, robotTheta)
    #plt.scatter(robotX, robotY, marker='x')
    
    X1= get_world_coords(robotX, robotY, robotTheta, lidar_data)
    #plot_world_coords(X, Y, 'red')
    
    plot_vars = [X1, robotX, robotY, robotTheta, walls]
    return  [robotX, robotY, robotTheta, unrobotX, unrobotY, unrobotTheta, P, plot_vars]#, lidar_world]

    

def get_pred_pos(SL, SR,  robotX, robotY, robotTheta):
    
    
    b = robot_params.pioneer_track_width
    
    delta_trans = (SL + SR) / 2 
    
    robotX = robotX + delta_trans * math.cos(robotTheta + (SR- SL) / (2 * b))
    robotY = robotY + delta_trans * math.sin(robotTheta + (SR- SL) / (2 * b))
    robotTheta = robotTheta + (SR - SL) / ( b)
    
    return [robotX, robotY, robotTheta]


                
def get_dist(cylinder, world_cylinder):
    
    dist = math.sqrt( (cylinder[0] - world_cylinder[0])**2 + (cylinder[1] - world_cylinder[1])**2 )
    
    return dist
    
def get_world_coords(robotX, robotY, robotTheta, lidar_data):

    
    coords = []

    for i in range(len(lidar_data)):

        angle = robotTheta + lidar_data[i][0]

        r = lidar_data[i][1]
        x = robotX + r * math.cos(angle)
        y = robotY + r * math.sin(angle)

        coords.append([x,y])
              
    return coords
        


def process_lidar_data(lidar_data, step_size):
    
    lidar_data_processed = []
    #Start angle
    angle = 0
    
    #num of lidar points  
    num_points = len(lidar_data)
    
    for i in range(num_points):        
        r = lidar_data[i]
        if r == np.inf:
            angle += step_size 
            continue
        lidar_data_processed.append([angle, lidar_data[i]])       
        angle+= step_size
    return lidar_data_processed