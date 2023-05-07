
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
import robot_params
import time 
import message_filters
import localization_constants
import wall_localization
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf
from tf.transformations import quaternion_matrix, quaternion_from_matrix, compose_matrix, translation_from_matrix, inverse_matrix, identity_matrix, euler_from_quaternion

#setup paths
cwd_path = os.path.dirname(os.path.abspath(__file__)) #gettting the parent directory path

#Setup plot
plt.ion()# seeting up interactive mode

#lists for stroing x position y position and orientation
robot_position_x = []
robot_position_y = []
robot_orientation = []


# Localized robot Parameters
robotTheta_lidar = np.pi/2
robotX_lidar = 1.2
robotY_lidar = 1.5

robotX_centre = robotX_lidar - 0.2 *  np.cos(robotTheta_lidar)
robotY_centre = robotY_lidar - 0.2 *  np.sin(robotTheta_lidar) 
robotTheta_centre = robotTheta_lidar 


P = np.zeros(shape=(3,3))
P_odom = np.zeros(shape=(3,3))
#  Pioneer Params
p_X = 1.2
p_Y = 1.5
p_Theta = np.pi/2


wallx = []
wally = [] 

trajectory_x = []
trajectory_y = []

SL = 0
SR = 0


px_old = 1.2
py_old = 1.5
ptheta_old = np.pi/2


def plot_data2Str(robotX, robotY, robotTheta, p_X, p_Y, p_Theta, P, corr_walls, walls, coords):
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
    
    X, Y = coords
    for x in X:
        plot_vars += str(x) + ' '
    plot_vars += '\n'
    
    for y in Y:
        plot_vars += str(y) + ' '
    plot_vars += '\n'
    return plot_vars

def odom_to_wheeldist(delta_x, delta_y, delta_theta):


    delta_d = np.sqrt(delta_x **2 + delta_y **2 ) 
    
    sr = delta_d + delta_theta * robot_params.pioneer_track_width
    sl = delta_d - delta_theta * robot_params.pioneer_track_width
    
    return [sr, sl]
    
    
    

def odom_callback(data):
    print('odom callback')
    
    global robotX_centre
    global robotY_centre
    global robotTheta_centre
    
    global robotX_lidar
    global robotY_lidar
    global robotTheta_lidar
    
    global p_X
    global p_Y
    global p_Theta
    global SR
    global SL
    
    global P_odom
    
    phi = -np.pi/2
    T = np.array([[np.cos(phi),np.sin(phi), 1.2],[-np.sin(phi), np.cos(phi), 1.5],[0, 0, 1]])
    p_Xnew = data.pose.pose.position.x 
    p_Ynew = data.pose.pose.position.y 
    p_Thetanew = data.pose.pose.orientation.x
    
    corrected = np.matmul(T, np.array([[p_Xnew],[p_Ynew],[1]]))
    
    p_Xnew = corrected[0, 0]
    p_Ynew = corrected[1, 0]
    p_Thetanew = p_Thetanew + np.pi/2
    
# =============================================================================
#     print('p_Xnew', p_Xnew)
#     print('p_Ynew', p_Ynew)
#     print('p_Thetanew', p_Thetanew)
# =============================================================================
    
    delta_x = p_Xnew - p_X
    delta_y = p_Ynew - p_Y
    delta_theta = p_Thetanew - p_Theta
    
    
    p_X = p_Xnew
    p_Y = p_Ynew
    p_Theta = p_Thetanew

def april_callback(data):
    global robotX_centre
    global robotY_centre
    global robotTheta_centre
    global P
    
    
    print("before apriltag", robotX_centre, robotY_centre, robotTheta_centre)
    robotX_camera = data.pose.pose.position.x
    robotY_camera = data.pose.pose.position.y
    
    
    m = quaternion_matrix([data.pose.pose.orientation.x ,data.pose.pose.orientation.y ,data.pose.pose.orientation.z ,data.pose.pose.orientation.w ])
    theta = euler_from_quaternion(m)[2]
    
    robotTheta_centre = theta
    robotX_centre = robotX_camera - 0.2 * np.cos(theta)
    robotY_centre = robotY_camera - 0.1 * np.cos(theta)
    
    T1 = np.array([[np.cos(theta), -np.sin(theta), robotX_camera],[np.sin(theta), np.cos(theta), robotY_camera],[0, 0 , 1]])
    T2 = np.array([[1,0, -0.2],[0,1, -0.1],[0, 0 , 1]])
    out = np.matmul(T1,np.matmul(T2, np.array([0, 0, 1]).reshape((3,1))))
    
    robotX_centre = out[0].squeeze()
    robotY_centre = out[1].squeeze()
    P = np.eye(3)* 0.02
    

def pose_callback(data):
    print("callback")
    

    tick = time.time()
    #acessing the required global variables 
    global robot_position_x
    global robot_position_y

    global robotX_centre
    global robotY_centre
    global robotTheta_centre
    
    global robotX_lidar
    global robotY_lidar
    global robotTheta_lidar
    
    global unrobotX
    global unrobotY
    global unrobotTheta
    global odometry_data
    global P

    global lines
    global pts
    global p_X
    global p_Y
    global p_Theta
    
    global SR
    global SL
    
    global px_old
    global py_old
    global ptheta_old
    
    
    #finding th erequired points to be plotted 
    X = []
    Y = []
    odometry_data =[0,SR,SL]

    lidar_data = data.ranges
    step_size = data.angle_increment
    
    delta_x = p_X - px_old
    delta_y = p_Y - py_old
    delta_theta = p_Theta - ptheta_old

    SL, SR = odom_to_wheeldist(delta_x, delta_y, delta_theta)
    print('SR, SL')
    print(SR, SL)
    
    px_old = p_X
    py_old = p_Y
    ptheta_old = p_Theta
    
    
    robotX_lidar, robotY_lidar, robotTheta_lidar, P, plot_vars=  wall_localization.localize(lidar_data, step_size, odometry_data, robotX_lidar, robotY_lidar, robotTheta_lidar, P)
    robotX_centre = robotX_lidar - 0.2 * np.cos(robotTheta_lidar)
    robotY_centre = robotY_lidar - 0.2 * np.sin(robotTheta_lidar)
    robotTheta_centre = robotTheta_lidar
    


    tock = time.time()
    if plot_vars !=[]:

        X1,robotX, robotY, robotTheta, corr_walls, walls = plot_vars
# =============================================================================
#         print('robotPos')
# =============================================================================
        print(robotX_centre, robotY_centre, robotTheta_centre)
# =============================================================================
#         print('P3AT')
# =============================================================================
        print(p_X, p_Y, p_Theta)
        plot_data = plot_data2Str(robotX_centre,robotY_centre, robotTheta_centre, p_X, p_Y, p_Theta, P,corr_walls, walls, X1)
        pub.publish(plot_data)
        
    else:
    	print("publishing without localizing")
    	
    	
if __name__ == '__main__':

    #intialising a node for the vizualisation part 
    rospy.init_node('rover_visualisation', anonymous=True)
    
    pub = rospy.Publisher("plot_data", String, queue_size=1)
    #subscribing the required topic and updating its callback function 
    laser_sub = rospy.Subscriber("/scan", LaserScan, pose_callback,queue_size=1)
    pos_sub = rospy.Subscriber("/RosAria/pose", Odometry, odom_callback,queue_size=1)
    april_sub = rospy.Subscriber("/tag_detections", PoseWithCovarianceStamped, april_callback, queue_size = 1)
    plt.show(block=True)
    
    rospy.spin()
