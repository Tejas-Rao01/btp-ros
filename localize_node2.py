#!/usr/bin/env python3
#importing the necessary Libraries required 
import matplotlib.pyplot as plt #for plotting 
import numpy as np # the numerical python library
import rospy #python library for ros
import os # python library for system file path
from geometry_msgs.msg import Twist #importing the messgae for publishing 
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import wall_localization
import time 
import message_filters
#setup paths
cwd_path = os.path.dirname(os.path.abspath(__file__)) #gettting the parent directory path

#Setup plot
plt.ion()# seeting up interactive mode

#lists for stroing x position y position and orientation
robot_position_x = []
robot_position_y = []
robot_orientation = []


# Localized robot Parameters
robotX = -0.2
robotY = 0 
robotTheta = 0

unrobotX = robotX
unrobotY = 0
unrobotTheta = 0 
P = np.zeros(shape=(3,3))

#  Pioneer Params
p_X = 0
p_Y = 0
p_Theta = 0


wallx = []
wally = [] 

trajectory_x = []
trajectory_y = []

def plot_data2Str(robotX, robotY, robotTheta, p_X, p_Y, p_Theta, P, corr_walls, walls):
    plot_vars = f'{robotX} {robotY} {robotTheta} \n'
    plot_vars += f'{p_X} {p_Y} {p_Theta} \n'
    P_str = ''
    for i in range(3):
        for j in range(3):
            P_str += str(P[i, j]) + ' '
    plot_vars += P_str
    plot_vars += '\n'
    plot_vars += str(len(corr_walls)) + ' '
    plot_vars += str(len(walls))
    plot_vars += '\n'
    for corr_wall in corr_walls:
        p1, p2 = corr_wall
        p_str = f'{p1[0]} {p1[1]} {p2[0]} {p2[1]} '
        plot_vars += p_str
        plot_vars += '\n'

    for wall in walls:
        p1, p2 = wall
        p_str = f'{p1[0]} {p1[1]} {p2[0]} {p2[1]} '
        plot_vars += p_str
        plot_vars += '\n'
    return plot_vars
    


def callback(laser_data, odom_data):
    print('odom callback')
    
    global robotX
    global robotY 
    global robotTheta 
    global p_X
    global p_Y
    global p_Theta
    
    p_Xnew = odom_data.pose.pose.position.x
    p_Ynew = odom_data.pose.pose.position.y
    p_Thetanew = odom_data.pose.pose.orientation.x
    
    delta_x = p_Xnew - p_X
    delta_y = p_Ynew - p_Y
    delta_theta = p_Thetanew - p_Theta
    
    robotTheta = robotTheta + delta_theta
    robotX = robotX + delta_x + 0.2 * np.cos(robotTheta)
    robotY = robotY + delta_y + 0.2 * np.sin(robotTheta)
    
    p_X = p_Xnew
    p_Y = p_Ynew
    p_Theta = p_Thetanew
    
    print('robotX robotY ', robotX, robotY)
    print('deltax , delta y', delta_x, delta_y)
    
    
    #acessing the required global variables 
    global robot_position_x
    global robot_position_y

    
    global unrobotX
    global unrobotY
    global unrobotTheta
    global odometry_data
    global P

    global lines
    global pts
    
    #finding th erequired points to be plotted 
    X = []
    Y = []
    odometry_data =[0,0,0]
    P = np.eye(3)
    lidar_data = laser_data.ranges
    step_size = laser_data.angle_increment
    
# =============================================================================
#     print('robotx before', robotX)
# =============================================================================
    robotX, robotY, robotTheta, P, plot_vars=  wall_localization.localize(lidar_data, step_size, odometry_data, robotX, robotY, robotTheta, unrobotX, unrobotY, unrobotTheta, P)
# =============================================================================
#     print('robotX after', robotX)
# =============================================================================
# =============================================================================
#     print('PX, PY', p_X, p_Y)
# =============================================================================

    if plot_vars !=[]:
# =============================================================================
#         print("publishing")
# =============================================================================
        X1,robotX, robotY, robotTheta, corr_walls, walls = plot_vars
        plot_data = plot_data2Str(robotX, robotY, robotTheta, p_X, p_Y, p_Theta, P,corr_walls, walls)
        pub.publish(plot_data)
        
    else:
    	print("publishing without localizing")
    	
    	
if __name__ == '__main__':

    #intialising a node for the vizualisation part 
    rospy.init_node('rover_visualisation', anonymous=True)
    
    pub = rospy.Publisher("plot_data", String, queue_size=1)
    #subscribing the required topic and updating its callback function 
    laser_sub = message_filters.Subscriber("/scan", LaserScan)
    pos_sub = message_filters.Subscriber("/RosAria/pose", Odometry)
    ts = message_filters.ApproximateTimeSynchronizer([laser_sub, pos_sub], 10,slop=3)
    ts.registerCallback(callback)
    plt.show(block=True)
    
    rospy.spin()
