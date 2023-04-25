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

def localize(lidar_data, step_size, odometry_data, robotX, robotY, robotTheta, P):
    
    
    lidar_data = process_lidar_data(lidar_data, step_size)
    #plt.scatter(robotX, robotY,c='red', marker='x')
    # Check 
    [odomTime, SL, SR] = odometry_data
        

    #walls = []
    robotX, robotY, robotTheta, P, corresponded_walls, walls = wall_ekf.kalman_filter([robotX, robotY, robotTheta], lidar_data, P, SL, SR)

    #print("robotX, robotY, robotTheta", robotX, robotY, robotTheta)
    #plot_walls(walls, robotX, robotY, robotTheta)
    #plt.scatter(robotX, robotY, marker='x')
    X1 = [[0],[0]]
    
    
     
    plot_vars = [X1, robotX, robotY, robotTheta, corresponded_walls, walls]
    return  [robotX, robotY, robotTheta, P, plot_vars]#, lidar_world]

def plot_world_coords(coords):
    X = coords[0]
    Y = coords[1]
    for x,y in zip(X,Y):
        
        plt.scatter(x,y, c='r')

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
 
    xs = []
    ys = []

    for i in range(0,len(lidar_data),4):
        angle = robotTheta + lidar_data[i][0]
        r = lidar_data[i][1]
        x = robotX + r * math.cos(angle)
        y = robotY + r * math.sin(angle)
        xs.append(x)
        ys.append(y)
    
    coords = [xs, ys]
    return coords

    
    
def get_coords_wrt_robot(robotX, robotY, robotTheta, lidar_data):

    xs = []
    ys = []
    for i in range(len(lidar_data)):

        angle = lidar_data[i][0]

        r = lidar_data[i][1]
        x = 0 + r * math.cos(angle)
        y = 0 + r * math.sin(angle)

        xs.append(x)
        ys.append(y)
    coords = [xs, ys]              
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



def plot_data2Str(robotX, robotY, robotTheta, P, walls):
    plot_vars = f'{robotX} {robotY} {robotTheta} \n'
    P_str = ''
    for i in range(3):
        for j in range(3):
            P_str += str(P[i, j]) + ' '
    plot_vars += P_str
    plot_vars += '\n'
    for wall in walls:
        p1, p2 = wall
        p_str = f'{p1[0]} {p1[1]} {p2[0]} {p2[1]} '
        plot_vars += p_str
        plot_vars += '\n'
    return plot_vars

def Str2plot_data(plot_str):
    #print(plot_str.data)
    lines = plot_str.data.split('\n')

    P = np.zeros(shape=(3,3))
    line_list = []
    for line_id, line in enumerate(lines):
        if line_id == 0:
            robotX, robotY, robotTheta = list(map(float, line.split()))
        if line_id == 1:
            P_data = list(map(float, line.split()))
            for i in range(3):
                for j in range(3):
                    P[i, j] = P_data[i*3+ j]
        
        if line_id > 1:
            if line.split():
                p1, p2 = [0,0], [0,0]
                p1[0], p1[1], p2[0], p2[1] = list(map(float, line.split()))
                line_list.append([p1, p2])
                
                
    return robotX, robotY, robotTheta, P, line_list

def get_error_ellipse(covariance):
        """Return the position covariance (which is the upper 2x2 submatrix)
           as a triple: (main_axis_angle, stddev_1, stddev_2), where
           main_axis_angle is the angle (pointing direction) of the main axis,
           along wprint('alpha_inn',alpha_inn)hich the standard deviation is stddev_1, and stddev_2 is the
           standard deviation along the other (orthogonal) axis."""
        eigenvals, eigenvects = np.linalg.eig(covariance[0:2,0:2])
        angle = np.arctan2(eigenvects[1,0], eigenvects[0,0])
        return (angle, np.sqrt(eigenvals[0]), np.sqrt(eigenvals[1]))

def ellipse_data(angle, a, b):
    
    thetas = np.linspace(0, 2*np.pi, 40)
    x = a*np.cos(thetas)
    y = b*np.sin(thetas)    
    
    coord = np.array([[x],[y]]).reshape(2, 40)
    rot= np.array([[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]])    
    
    coord = np.matmul(rot, coord)

    x = coord[0,:]
    y = coord[1,:]
    return x, y

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
    
def compute_derivative(lidar_data):
        
    # Computing Derivatives 
    derivative = []
    for i in range(1,len(lidar_data)-1):
        l = lidar_data[i-1][1]
        r = lidar_data[i+1][1]
        d = (r- l)/2
        derivative.append(d)
          
    return derivative

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
                print('avg_angle ',avg_angle *180 / math.pi)
                theta = robotTheta + avg_angle -math.pi/2
                print('localize2 x: ' ,robotX)
                print('localize2 y: ' ,robotY)
                print('localize2 t: ' ,robotTheta)
                
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

def plot_map():
    
    plt.plot([2.4,2.4],[-3,3], color='green')
    plt.plot([-0.5,2.4],[-2.83,-2.83], color='green')    
    

if __name__ =="__main__":
    
    
    data = np.load('lidar_scan.npy')

    plot_map()
    lidar_data = data[5,3:]
    step_size = data[5,2]

    robotX, robotY, robotTheta = [0,0,0]
    print('robot pos before : ', robotX, robotY, robotTheta)
    robotPos = [robotX, robotY, robotTheta]
    unrobotX, unrobotY, unrobotTheta = robotX, robotY, robotTheta
    
    SL, SR = 0,0
    P = np.eye(3)
    
    odometry_data = [0, SL, SR]
    
    robotX, robotY, robotTheta, P, plot_vars = localize(lidar_data, step_size, odometry_data, robotX, robotY, robotTheta, P)
    X1,robotX, robotY, robotTheta, corr_walls, walls = plot_vars

    print('robot pos after : ', robotX, robotY, robotTheta)
    plt.scatter(robotX, robotY, c='g')
    
    heading_x = [robotX, robotX + 1*np.cos(robotTheta)]
    heading_y = [robotY, robotY + 1*np.sin(robotTheta)]
    plt.plot(heading_x, heading_y)
    
    
    plt.xlim(-6,6)
    plt.ylim(-6,6)
    transformation_mat = np.array([ [np.cos(robotTheta), -np.sin(robotTheta), robotX],[np.sin(robotTheta), np.cos(robotTheta), robotY],[0,0,1]])
    for wall in walls:
        p1, p2 = wall
        p1 = list(p1)
        p2 = list(p2)
        p1.append(1)
        p2.append(1)
        p1 = np.matmul(transformation_mat, np.array(p1).reshape((3,1)))
        p2 = np.matmul(transformation_mat, np.array(p2).reshape( (3,1)))

        x = [p1[0][0], p2[0][0]]
        y = [p1[1][0], p2[1][0]]

        plt.plot(x, y,'black')
        
    for wall in corr_walls:
        p1, p2 = wall
        p1 = list(p1)
        p2 = list(p2)
        p1.append(1)
        p2.append(1)
        p1 = np.matmul(transformation_mat, np.array(p1).reshape((3,1)))
        p2 = np.matmul(transformation_mat, np.array(p2).reshape( (3,1)))

        x = [p1[0][0], p2[0][0]]
        y = [p1[1][0], p2[1][0]]

        plt.plot(x, y,color='blue', markersize=10)
    
    lidar_data = process_lidar_data(lidar_data, step_size)
    [cylinders, derivatives] = detect_cyls(lidar_data, robotX, robotY, robotTheta)
    print('cylinders',cylinders)
    print('len cylinder', len(cylinders))
    
    lidar_data = clean_data(lidar_data, derivatives)
    coords = get_coords_wrt_robot(robotX, robotY, robotTheta, lidar_data)
    
    plot_world_coords(coords)
    plt.axis('equal')
    
    
    