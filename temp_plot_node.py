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
import robot_params
import time 

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

wall_begining = [[-2,-2],[-2,2],[2,2],[2,-2]]
wall_end = [[-2,2],[2,2],[2,-2],[-2,-2]]

for start_coord, end_coord in zip(wall_begining, wall_end):
    x = [start_coord[0], end_coord[0]]
    y = [start_coord[1], end_coord[1]]
    
    ax1.plot(x,y,'green')
X_cor = [0.0, 1.0]
Y_cor = [0.0, 0.0]
heading_1, = ax1.plot(X_cor, Y_cor,'b-') # plotting the intial heading from start to another point 
cov_ellipse, = ax1.plot(X_cor, Y_cor, 'b-')


lines = ax1.plot(np.empty((0, 100)), np.empty((0, 100)),color='black', lw=2)
corr_lines = ax1.plot(np.empty((0, 100)), np.empty((0, 100)),color='blue', lw=2)
ax1.set(xlim=(-1,50), ylim=(-1,50))
traj, = ax1.plot([0, 0], [0, 0], 'blue')
p_traj, = ax1.plot([0, 0], [0, 0], 'purple')
robotX = 0
robotY = 0
robotTheta = np.pi

wall_scatter = ax1.scatter(3,3, c='r')
p_X = robotX
p_Y = 0
p_Theta = 0 

wallx = []
wally = [] 


trajectory_x = []
trajectory_y = []

p_trajectory_x = []
p_trajectory_y = []


SR = 0 
SL = 0

plt.axis('equal')
robot_pos_plt = ax1.scatter(robotX,robotY,s=30,marker='^',c='g')
def Str2plot_data(plot_str):
    #print(plot_str.data)
    lines = plot_str.data.split('\n')

    P = np.zeros(shape=(3,3))
    odom_P = np.zeros(shape=(3,3))
    corr_line_list = []
    line_list = []
    for line_id, line in enumerate(lines):
        if line_id == 0:
            robotX, robotY, robotTheta = list(map(float, line.split()))
            continue
        if line_id == 1:
            p_X, p_Y, p_Theta = list(map(float, line.split()))
            continue
        
        if line_id == 2:
            P_data = list(map(float, line.split()))
            for i in range(3):
                for j in range(3):
                    P[i, j] = P_data[i*3+ j]
            continue
        
        if line_id == 3:
            a, b = list(map(int, line.split()))
            continue
        if line_id > 3 and line_id <= 3 + a:
            if line.split():
                p1, p2 = [0,0], [0,0]
                p1[0], p1[1], p2[0], p2[1] = list(map(float, line.split()))
                corr_line_list.append([p1, p2])
            continue 
        
        if line_id > 3+a and line_id <= 3 + a + b:
            if line.split():
                p1, p2 = [0,0], [0,0]
                p1[0], p1[1], p2[0], p2[1] = list(map(float, line.split()))
                line_list.append([p1, p2])
            continue    
        if line_id == 4 + a + b:
            Xs = list(map(float, line.split()))
            continue
        
        if line_id == 5 + a + b:
            Ys = list(map(float, line.split()))
            continue 
# =============================================================================
#         if line_id == 6 + a + b:
#            P_data = list(map(float, line.split()))
#            for i in range(3):
#                for j in range(3):
#                    odom_P[i, j] = P_data[i*3+ j]
# =============================================================================
   

        
    return robotX, robotY, robotTheta,p_X, p_Y, p_Theta, P, corr_line_list, line_list, Xs, Ys

def get_error_ellipse(covariance):
        """Return the position covariance (which is the upper 2x2 submatrix)
           as a triple: (main_axis_angle, stddev_1, stddev_2), where
           main_axis_angle is the angle (pointing direction) of the main axis,
           along which the standard deviation is stddev_1, and stddev_2 is the
           standard deviation along the other (orthogonal) axis."""
        eigenvals, eigenvects = np.linalg.eig(covariance[0:2,0:2])
        angle = np.arctan2(eigenvects[1,0], eigenvects[0,0])
        return (angle, np.sqrt(eigenvals[0]), np.sqrt(eigenvals[1]))

def ellipse_data(angle, a, b, robotX, robotY, robotTheta):
    
    thetas = np.linspace(0, 2*np.pi, 40)
    x = a*np.cos(thetas)
    y = b*np.sin(thetas)    
    
    coord = np.array([[x],[y]]).reshape(2, 40)
    rot= np.array([[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]])    
    
    coord = np.matmul(rot, coord)


    
    x = coord[0,:] + robotX
    y = coord[1,:] + robotY
    return x, y
    

    
def pose_callback(data):
    print("callback")
    #acessing the required global variables 
    
    """
    variables to unpack - robot_X, robotY, robotTheta, P, walls
    """
    
    global robot_position_x
    global robot_position_y
    global ax1
    global robotX
    global robotY
    global robotTheta
    global unrobotX
    global unrobotY
    global unrobotTheta
    global odometry_data
    global P
    global heading_1 
    global lines
    global corr_lines
    global pts
    global traj
    global trajectory_x
    global trajectory_y
    global cov_ellipse
    global robot_pos_plt
    
    global p_trajectory_x 
    global p_trajectory_y
    
    global wall_scatter
    global SR
    global SL
    
    #finding th erequired points to be plotted 
    X = []
    Y = []
    odometry_data =[0,0,0]
    
    robotX, robotY, robotTheta,p_X, p_Y, p_Theta, P,corr_walls,  walls, Xs, Ys = Str2plot_data(data)
    
    trajectory_x.append(robotX)
    trajectory_y.append(robotY)

    p_trajectory_x.append(p_X)
    p_trajectory_y.append(p_Y)

    transformation_mat = np.array([ [np.cos(robotTheta), -np.sin(robotTheta), robotX],[np.sin(robotTheta), np.cos(robotTheta), robotY],[0,0,1]])
    tick= 0
    time.sleep(0.1)
# =============================================================================
#     
#     print('walls')
#     print(walls)
#     
#     print('corr walls')
#     print(corr_walls)
#     
# =============================================================================
    
    if walls == None:
        print('no walls')
        for i in range(len(lines)):
            lines[i].set_data(0,0)
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

        lines[tick].set_data(x,y)
        tick +=1
        if tick >= min(len(walls), len(lines)):
            break
    tick = 0
    if corr_walls == []:
        print('no corr walls')
        for i in range(len(corr_lines)):
            corr_lines[i].set_data(0,0)
            
    
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

        corr_lines[tick].set_data(x,y)
        tick +=1
        if tick >= min(len(corr_walls), len(corr_lines)):
            corr_lines[tick].set_data(0,0)
        
    angle, a, b = get_error_ellipse(P)
    x, y = ellipse_data(angle, a, b,robotX, robotY, robotTheta)
    
    heading_x = [robotX, robotX + 1*np.cos(robotTheta)]
    heading_y = [robotY, robotY + 1*np.sin(robotTheta)]
    heading_1.set_xdata(heading_x)
    heading_1.set_ydata(heading_y)
    robot_pos_plt.set_offsets([robotX, robotY])
    
    
    cov_ellipse.set_xdata(x)
    cov_ellipse.set_ydata(y)
    
    coords = []
    for x, y in zip(Xs, Ys):
        coords.append([x,y])
    
    
    wall_scatter.set_offsets(coords)

    traj.set_data(trajectory_x, trajectory_y)
    p_traj.set_data(p_trajectory_x, p_trajectory_y)
    
    
    print('ellipse data', x, y)
    print('robotX, robotY, robotTheta :', robotX, robotY, robotTheta)
    time.sleep(0.1)

    print('p_X, p_Y',p_X, p_Y)
                
if __name__ == '__main__':

    #intialising a node for the vizualisation part 
    rospy.init_node('rover_visualisation', anonymous=True)

    #subscribing the required topic and updating its callback function 
    rospy.Subscriber("/plot_data", String, pose_callback,queue_size=1)
    plt.xlim(-1, 40)
    plt.ylim(-1,50)
    plt.axis('equal')
    plt.show(block=True)
    
    rospy.spin()
