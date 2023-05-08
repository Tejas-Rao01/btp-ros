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


# Start pos of robot
robotStart_pos = [1.1,1.5, np.pi/2]


# Localized robot Parameters
robotTheta_centre = robotStart_pos[2]
robotX_centre = robotStart_pos[0]
robotY_centre = robotStart_pos[1]

robotX_lidar = robotX_centre + 0.2 *  np.cos(robotTheta_centre)
robotY_lidar = robotY_centre + 0.2 *  np.sin(robotTheta_centre) 
robotTheta_lidar = robotTheta_centre


P = np.eye(3)*1
P_odom = np.eye(3)*1



#  Pioneer Params
p_X =  robotStart_pos[0]
p_Y =  robotStart_pos[1]
p_Theta =  robotStart_pos[2]


wallx = []
wally = [] 

trajectory_x = []
trajectory_y = []

SL = 0
SR = 0

april_tick = 0

px_old = robotStart_pos[0]
py_old = robotStart_pos[1]
ptheta_old = robotStart_pos[2]


cum_dl = 0
cum_do = 0 

April = False

robotX_centre_apr = robotStart_pos[0]
robotY_centre_apr = robotStart_pos[1]
robotTheta_centre_apr = robotStart_pos[2]

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

def odom_to_wheeldist(delta_x, delta_y, delta_theta, robotTheta):


    theta_not = delta_theta / 2 + robotTheta
    a = (delta_x+ delta_y ) * 2 / (np.cos(theta_not) + np.sin(theta_not))
    b = delta_theta * robot_params.pioneer_track_width
    sr = (a + b) / 2 
    sl = (a - b) / 2
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
    
    phi = -robotStart_pos[2]
    
    T = np.array([[np.cos(phi),np.sin(phi), robotStart_pos[0]],[-np.sin(phi), np.cos(phi), robotStart_pos[1]],[0, 0, 1]])
    p_Xnew = data.pose.pose.position.x 
    p_Ynew = data.pose.pose.position.y 


    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    p_Thetanew = np.arctan2(t3, t4)
    
    corrected = np.matmul(T, np.array([[p_Xnew],[p_Ynew],[1]]))
    
    p_Xnew = corrected[0, 0]
    p_Ynew = corrected[1, 0]
    
    

    p_X = p_Xnew
    p_Y = p_Ynew
    p_Theta = p_Thetanew

def april_callback(data):
    global robotX_centre_apr
    global robotY_centre_apr
    global robotTheta_centre_apr
    global P
    global april_tick

    global April
    
    if april_tick != 15:
        april_tick += 1
        return 
    else:
        april_tick = 0
        April = True
        print("before apriltag", robotX_centre, robotY_centre, robotTheta_centre)
        robotX_camera = data.pose.pose.position.x
        robotY_camera = data.pose.pose.position.y
        
        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        z = np.arctan2(t3, t4)
        theta = z + np.pi
        robotTheta_centre_apr  = theta
        
        T1 = np.array([[np.cos(theta), -np.sin(theta), robotX_camera],[np.sin(theta), np.cos(theta), robotY_camera],[0, 0 , 1]])
        T2 = np.array([[1,0, -0.2],[0,1, -0.1],[0, 0 , 1]])
        out = np.matmul(T1,np.matmul(T2, np.array([0, 0, 1]).reshape((3,1))))
        
        robotX_centre_apr = out[0].squeeze()
        robotY_centre_apr = out[1].squeeze()
        P = np.eye(3)* 0.5
        
        

        print("after apriltag", robotX_centre, robotY_centre, robotTheta_centre)
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
    
    global April
    
    global robotX_centre_apr
    global robotY_centre_apr
    global robotTheta_centre_apr
    #finding th erequired points to be plotted 

    
    lidar_data = data.ranges
    step_size = data.angle_increment
    
    delta_x = p_X - px_old
    delta_y = p_Y - py_old
    delta_theta = p_Theta - ptheta_old

    SL, SR = odom_to_wheeldist(delta_x, delta_y, delta_theta, robotTheta_centre)
    odometry_data =[0,SR,SL]


    px_old = p_X
    py_old = p_Y
    ptheta_old = p_Theta
    
    if April == True:
        print('april correction')
        robotX_centre = robotX_centre_apr
        robotY_centre = robotY_centre_apr
        robotX_lidar = robotX_centre_apr + 0.2 * np.cos(robotTheta_lidar)
        robotY_lidar = robotY_centre_apr + 0.2 * np.sin(robotTheta_lidar)
        robotTheta_centre = robotTheta_centre_apr
        robotTheta_lidar = robotTheta_centre
        April = False
        robotX_lidar = robotX_centre 
        plot_data = plot_data2Str(robotX_centre,robotY_centre, robotTheta_centre, p_X, p_Y, p_Theta, P,[], [], [[0], [0]])
        pub.publish(plot_data)
        return 
        
    print('april before ekf', April)
    robotX_lidar, robotY_lidar, robotTheta_lidar, P, plot_vars=  wall_localization.localize(lidar_data, step_size, odometry_data, robotX_lidar, robotY_lidar, robotTheta_lidar, P)
    robotX_centre = robotX_lidar - 0.2 * np.cos(robotTheta_lidar)
    robotY_centre = robotY_lidar - 0.2 * np.sin(robotTheta_lidar)
    robotTheta_centre = robotTheta_lidar
    print('april after ekf', April)

    
    tock = time.time()
    if plot_vars !=[]:

        X1,robotX, robotY, robotTheta, corr_walls, walls = plot_vars
        P = np.eye(3)* 0.5
# =============================================================================
#         print('P3AT')
# =============================================================================

        plot_data = plot_data2Str(robotX_centre,robotY_centre, robotTheta_centre, p_X, p_Y, p_Theta, P,corr_walls, walls, X1)
        pub.publish(plot_data)
        
    else:

        plot_data = plot_data2Str(robotX_centre,robotY_centre, robotTheta_centre, p_X, p_Y, p_Theta, P,corr_walls, walls, X1)
        pub.publish(plot_data)
        print("publishing without localizing")
    	
    	
if __name__ == '__main__':

    #intialising a node for the vizualisation part 
    rospy.init_node('rover_visualisation', anonymous=True)
    
    pub = rospy.Publisher("plot_data", String, queue_size=1)
    #subscribing the required topic and updating its callback function 
    laser_sub = rospy.Subscriber("/scan", LaserScan, pose_callback,queue_size=1)
    pos_sub = rospy.Subscriber("/RosAria/pose", Odometry, odom_callback,queue_size=1)
    april_sub = rospy.Subscriber("/aprtag", PoseWithCovarianceStamped, april_callback, queue_size = 1)
    plt.show(block=True)
    
    rospy.spin()