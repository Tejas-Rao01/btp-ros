#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 23 10:51:45 2023

@author: shreyash
"""


import localization_constants 
import robot_params
import numpy as np 
import math
import matplotlib.pyplot as plt
import wall_detection
from scipy.linalg import block_diag
# Redefine robotPos to be a np array
def kalman_filter(robotPos, lidar_data, P, SL, SR):
    
    
    # Unpacking inputs
    [robotX, robotY, robotTheta] = robotPos
    
# =============================================================================
#     print('before ', robotX, robotY)
# =============================================================================
    # Differences 
    delta_D = (SR + SL ) / 2                                               
    delta_theta = (SR - SL) / robot_params.pioneer_track_width
    

    # get covariance matrices 
    Q = get_Q(SL,SR) 
    Fp = get_Fp(delta_D, robotTheta, delta_theta)
    Fu = get_Fu(delta_D, robotTheta, delta_theta)
    R = localization_constants.R
    
    [robotX_bar, robotY_bar, robotTheta_bar] = get_pred_pos(SL, SR, robotX, robotY, robotTheta)
    Pbar = np.matmul(np.matmul(Fp, P), np.transpose(Fp)) + np.matmul(np.matmul(Fu, Q), np.transpose(Fu))
    
    

    [cylinders, derivatives] = detect_cyls(lidar_data, robotX_bar, robotY_bar, robotTheta_bar)
# =============================================================================
#     print('cylinders',cylinders)
#     print('len cylinder', len(cylinders))
# =============================================================================
    
# =============================================================================
#     lidar_data = clean_data(lidar_data, derivatives)
# =============================================================================
    
# =============================================================================
#     lidar_data = clean_data(lidar_data, derivatives)
# =============================================================================
    wall_detector = wall_detection.wall_detection(lidar_data, robotX_bar, robotY_bar, robotTheta_bar)
    detected_walls = wall_detector.detected_walls(flag = False)
    
    walls = wall_detector.get_ls()

    if len(detected_walls) == 0:
        print('no walls detected')
        return robotX_bar, robotY_bar, robotTheta_bar, Pbar, [], []
    
    tick = -1

    innovation_stack = np.zeros(shape=(2*len(detected_walls),1))
    H_stack = np.zeros(shape=(2*len(detected_walls),3))
    R_stack = np.zeros(shape=(2*len(detected_walls), 2))
    wall_inds = []
    which_wall = []
# =============================================================================
#     
#     plot_scan(lidar_data, derivatives)
# =============================================================================
    
# =============================================================================
#     print()
#     print('---------------------')
#     print('beginning matching')
# =============================================================================

    for i in range(len(detected_walls)):   
        zi = np.array([[detected_walls[i][0]], [detected_walls[i][1]]])   
        flag, zhati, innovation, H, alpha, ind = get_corresponding_wall(zi, robotX_bar, robotY_bar, robotTheta_bar, Pbar, R)

        if not flag:
            continue
# =============================================================================
#         
#         
#         print('after correspondence')
#         print('zi', zi)
#         print('zhati', zhati)
#         print('innovation', innovation)
#         print('wall ind', i)
# =============================================================================
        which_wall.append(ind)
        tick += 1
        wall_inds.append(i)
        innovation_stack[tick*2:tick*2+2,:] = innovation
        H_stack[tick*2:tick*2 +2,:] = H
        R = block_diag(R, localization_constants.R)
        
        
# =============================================================================
#     print('len detected cyls', len(detected_cyls))
# =============================================================================
# =============================================================================
#     print(tick)    
# =============================================================================
    corresponded_walls = []

    if tick ==-1:
# =============================================================================
#         print(f'correspondence not found, {len(corresponded_walls)} walls matched')
#         print('--------------------------------')
# =============================================================================
        return [robotX_bar, robotY_bar, robotTheta_bar, Pbar,corresponded_walls, walls]
    
    
    H_stack = H_stack[0:tick*2+2,:]
    R_stack = R[0:tick*2+2,0:tick*2+2]

    innovation_stack = innovation_stack[0:tick*2+2,:]
    
    K,Sigma_inn = compute_kalman_gain(Pbar, H_stack, R_stack) 
    
# =============================================================================
#     print('H_stack')
#     print(H_stack)
#     
#     print('R_stack')
#     print(R_stack)
# 
#     print('innovatin_stack')
#     print(innovation_stack)    
#     
#     print('before', robotX, robotY)
# =============================================================================
    [robotX, robotY, robotTheta] = update_pos(robotX_bar, robotY_bar, robotTheta_bar, K, innovation_stack)
# =============================================================================
#     print('after', robotX_bar, robotY_bar)
# =============================================================================
    
    
    P = update_uncertainity(Pbar, K, Sigma_inn)
    
    robotTheta = robotTheta % (2 * math.pi)
    if robotTheta > math.pi:
        robotTheta = robotTheta - math.pi * 2
    if robotTheta < -math.pi:
        robotTheta = robotTheta + math.pi * 2 
    
    

     
    for ind in wall_inds:
        print('wall ind', ind)
        corresponded_walls.append(walls[ind])

# =============================================================================
#     print(f'correspondence found, {len(corresponded_walls)} walls matched')
#     print('--------------------------------')
#     
#     print('after',  robotX, robotY)
# =============================================================================
    
    print('returning')
    return [robotX, robotY, robotTheta, P, corresponded_walls, walls]

def clean_data(lidar_data, derivatives):
    temp_lidar = [lidar_data[0]]
    tick = 0
    for i in range(1, len(lidar_data)-1):
        if tick != 0:
            tick -=1
            continue 
        angle, r  = lidar_data[i]
        if derivatives[i-1] > 0.1 or derivatives[i-1] <-0.1:
            tick = 10
            continue
        temp_lidar.append([angle, r])
    
    lidar_data = temp_lidar
    return lidar_data
    

def detect_cyls(lidar_data, robotX, robotY, robotTheta):
    
    derivative = compute_derivative(lidar_data)
    cylinders = []
    start = False
    for i in range(len(derivative)):

        if derivative[i] < -localization_constants.cylinder_threshold_derivative :
            start = True
            avg_angle = 0
            n_indices = 0
            avg_depth = 0
            n_indices = 0
            avg_depth = 0 
            avg_indice = 0
            start = True
        if start == True and derivative[i] > localization_constants.cylinder_threshold_derivative \
            and n_indices > 0:
            avg_indice  = avg_indice / n_indices
            avg_angle = avg_angle / n_indices
            avg_depth = avg_depth / n_indices + localization_constants.cylinder_offset
            if avg_depth> 0.2:
# =============================================================================
#                 print('avg_angle ',avg_angle *180 / math.pi)
# =============================================================================
                theta = robotTheta + avg_angle -math.pi/2
# =============================================================================
#                 print('localize2 x: ' ,robotX)
#                 print('localize2 y: ' ,robotY)
#                 print('localize2 t: ' ,robotTheta)
#                 
# =============================================================================
                x = robotX + avg_depth * math.cos(theta)
                y = robotY + avg_depth * math.sin(theta)                
                cylinders.append([x, y, avg_depth, avg_angle])
            
            start = False
        if start == True:
            avg_angle += lidar_data[i+1][0]
          #  print('lidar data angle ',lidar_data[i+1][0])
            avg_indice += i
            n_indices += 1
            avg_depth += lidar_data[i+1][1]
        
    
    return [cylinders, derivative]

def plot_scan(lidar_data, derivatives):
    x = []
    y = []
    xder= []
    yder = []
    
    plt.figure()
    for i in range(len(lidar_data)):
        
        x.append(i)
        y.append(lidar_data[i][1])
        
        if i > 0 and i < len(lidar_data)-1:
            xder.append(i)
            yder.append(derivatives[i-1])
    
    plt.plot(x, y)
    plt.plot(xder, yder)
    plt.pause(0.005)
    plt.clf()
    
def compute_derivative(lidar_data):
        
    # Computing Derivatives 
    derivative = []
    for i in range(1,len(lidar_data)-1):
        l = lidar_data[i-1][1]
        r = lidar_data[i+1][1]
        d = (r- l)/2
        derivative.append(d)
          
    return derivative

def get_pred_pos(SL, SR,  robotX, robotY, robotTheta):
    
    
    b = robot_params.pioneer_track_width
    delta_trans = (SL + SR) / 2 

    robotX = robotX + delta_trans * math.cos(robotTheta + (SR- SL) / (2 * b))
    robotY = robotY + delta_trans * math.sin(robotTheta + (SR- SL) / (2 * b))
    
    robotTheta = robotTheta + (SR - SL) / ( b)
    
    return [robotX, robotY, robotTheta]

def update_uncertainity(Pbar, K, Sigma_inn):

    P =  np.subtract(Pbar,  np.matmul(np.matmul(K, Sigma_inn), np.transpose(K)))
    return P
    
def update_pos(robotX_bar, robotY_bar, robotTheta_bar, K, innovation_stack):
    
    robotPos = np.array([[robotX_bar], [robotY_bar], [robotTheta_bar]])
    robotPos = np.add(robotPos, np.matmul(K, innovation_stack))
    return [robotPos[0][0], robotPos[1][0], robotPos[2][0]]
    

def compute_kalman_gain(Pbar, H, R):
    Sigma_inn = np.add(np.matmul(np.matmul(H, Pbar), np.transpose(H)), R)
    T =   np.linalg.inv(Sigma_inn)
    K = np.matmul( np.matmul(Pbar, np.transpose(H)), T)
    
    return K, Sigma_inn


def get_Fp(delta_D, theta, delta_theta):
    
    Fp = np.array(  [[1, 0 , -delta_D * math.sin(theta + delta_theta / 2)], \
                     [0, 1, delta_D * math.cos(theta + delta_theta / 2)], \
                     [0, 0, 1] ]   )
    
    return Fp

def get_Fu(delta_D, theta, delta_theta):
    
    L = robot_params.pioneer_track_width    
    Fu = np.array(  [[1/2 * math.cos(theta + delta_theta /2) - delta_D / (2 * L) * math.sin(theta + delta_theta / 2), 1/2 * math.cos(theta + delta_theta /2) + delta_D / (2 * L) * math.sin(theta + delta_theta / 2)       ], \
                     [1/2 * math.sin(theta + delta_theta /2) + delta_D / (2 * L) * math.cos(theta + delta_theta / 2), 1/2 * math.sin(theta + delta_theta /2) - delta_D / (2 * L) * math.cos(theta + delta_theta / 2)       ], \
                     [1 / L, -1/L]    ] ) 
        
    return Fu


def get_obs(cyl_coords, robotX, robotY, robotTheta):
    
    dx  = cyl_coords[0] - robotX
    dy = cyl_coords[1] - robotY     
    dist = math.sqrt((cyl_coords[0] - robotX)**2 + (cyl_coords[1] - robotY)**2)
    alpha = math.atan2(dy, dx) - robotTheta + math.pi / 2 
    if alpha < -math.pi:
        alpha = 2 * math.pi + alpha
    if alpha > math.pi:
        alpha = alpha - math.pi * 2 
    
    return [np.array([[dist], [alpha]]), (dx, dy)]


def get_Q(SL, SR):
    k = localization_constants.model_cov_const
    Q = np.array([[k * abs(SR), 0], [0 , k * abs(SL)]])
    return Q        


def world2robot(alpha, r, robotX, robotY, robotTheta):
    alpha_dash = alpha  - robotTheta
    r_dash = r - (robotX * np.cos(alpha) + robotY * np.sin(alpha))
    return [alpha_dash, r_dash ]

def get_corresponding_wall(zi, robotX_bar, robotY_bar, robotTheta_bar,Pbar, R):
    
# =============================================================================
#     print('find corresponding wall')
#     print('measured wall parameters')
#    
#     
# =============================================================================
    ind = 0
    alpha_measured, r_measured = zi 
    
   
    
    
    zi = np.array(zi).reshape((2,1))
    max_dist = np.inf
    zhati = np.array([[], []])
    flag = False
# =============================================================================
#     print()
#     print(' --------------------------------------')
#     print('matching')
# =============================================================================
    for i in range(len(localization_constants.world_walls)):

        [alpha_world, r_world] = localization_constants.world_walls[i]
        [alpha_pred, r_pred] = world2robot(alpha_world, r_world, robotX_bar, robotY_bar, robotTheta_bar)
        if alpha_pred < 0:
            alpha_pred = 2*math.pi + alpha_pred
      
        H = np.array([[0, 0, -1],[-np.cos(alpha_world), -np.sin(alpha_world), 0]])
        Sigma_inn = np.add(np.matmul(np.matmul(H, Pbar), np.transpose(H)), localization_constants.R)
        alpha_pred = alpha_pred % (2 * np.pi)
        alpha_inn = alpha_measured - alpha_pred
        
# =============================================================================
#         print('checking with world wall')
#         print('measured', alpha_measured, r_measured)
#         print('world', alpha_pred, r_pred)
#         print('alpha_inn',alpha_inn)
# =============================================================================
        if alpha_inn > np.pi:
            alpha_inn -= np.pi * 2
        elif alpha_inn < -np.pi:
            alpha_inn += np.pi * 2 
# =============================================================================
#         print('alpha_inn',alpha_inn)   
# =============================================================================
        innovation = np.array([alpha_inn, r_measured - r_pred]).reshape((2,1))

        maha_dist =   np.matmul(np.transpose(innovation ), np.matmul(np.linalg.inv(Sigma_inn), innovation))
        if maha_dist.squeeze() < max_dist and maha_dist.squeeze() < 0.5:
# =============================================================================
#             print('maha distance', maha_dist)
#             
#             print('correspondence found')
#             print('measured', alpha_measured, r_measured)
#             print('world', alpha_pred, r_pred)
# =============================================================================
            flag = True
            max_dist = maha_dist.squeeze()
            zhati = np.array([alpha_pred, r_pred]).reshape((2,1))
            alpha = alpha_world
            inn = innovation
            H_final = H
            ind = i
    
    if flag:
        
        return True, zhati, inn, H_final, alpha, ind
    else:
        return False, -1, -1,-1,-1,-1
    
    
def get_H(alpha):   
    return np.array([[0, 0, -1],[-np.cos(alpha), -np.sin(alpha), 0]])

# convert robotframe to origin 

        






