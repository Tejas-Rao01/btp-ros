#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 16 13:04:51 2023

@author: shreyash
"""

#!/usr/bin/env python3
#importing the necessary Libraries required 
import matplotlib.pyplot as plt #for plotting 
import numpy as np # the numerical python library
import rospy #python library for ros
import os # python library for system file path
from geometry_msgs.msg import Twist #importing the messgae for publishing 
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import wall_localization
import time 
import math
#setup paths
cwd_path = os.path.dirname(os.path.abspath(__file__)) #gettting the parent directory path

#Setup plot
plt.ion()# seeting up interactive mode

#lists for stroing x position y position and orientation
robot_position_x=[]
robot_position_y=[]
robot_orientation =[]

#defining two axes and figures for plotting 
fig, ax1 = plt.subplots()

#plot robot


X_cor = [0.0, 1.0]
Y_cor = [0.0, 0.0]
plts = ax1.scatter(X_cor, Y_cor,c='r') # plotting the intial heading from start to another point 
ax1.set(xlim=(-8,8), ylim=(-8,8))

    
def pose_callback(data):
    print("callback")
    #acessing the required global variables 
    
    """
    variables to unpack - robot_X, robotY, robotTheta, P, walls
    """
    
    global plts
    #finding th erequired points to be plotted 
    
    
    lidar_data = data.ranges
    step_size = data.angle_increment
    lidar_data = process_lidar_data(lidar_data, step_size)
    coords = get_world_coords(0,0,0,lidar_data)
    plts.set_offsets(coords)
    time.sleep(0.1)


def get_world_coords(robotX, robotY, robotTheta, lidar_data):

    coords = []
    for i in range(len(lidar_data)):

        angle = robotTheta + lidar_data[i][0]

        r = lidar_data[i][1]
        x = robotX + r * math.cos(angle)
        y = robotY + r * math.sin(angle)

        coords.append([x,y])
              
    return coords

def get_coords(lidar_data):
    X = []
    Y = []
    for i in range(len(lidar_data)):

        angle = lidar_data[i][0]
        r = lidar_data[i][1]
        
        x = r * math.cos(angle)
        y = r * math.sin(angle)

        X.append(x)
        Y.append(y)
              
    return X, Y
    
    

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
                
if __name__ == '__main__':

    #intialising a node for the vizualisation part 
    rospy.init_node('rover_visualisation', anonymous=True)

    #subscribing the required topic and updating its callback function 
    rospy.Subscriber("/scan", LaserScan, pose_callback, queue_size=1)
    plt.show(block=True)
    
    rospy.spin()





