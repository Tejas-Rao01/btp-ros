import numpy as np 



def projection_point2line(point, m, c):
    
    x, y = point 
    if m != 0:
      m2 = -1/m 
      c2 = y - m2 * x
    
      intersection_x = - (c - c2) / (m - m2)
      intersection_y = m2 * intersection_x  + c2 
    else:
      intersection_x = x 
      intersection_y = c
    
    return intersection_x, intersection_y

def dist_p2p( p1, p2):

    x = p1[0] - p2[0]
    y = p1[1] - p2[1]

    return np.sqrt(x ** 2 + y ** 2)

def wallcoords_to_parametric(wall_beginning, wall_end):
    world_walls = []
    for start, end in zip(wall_beginning, wall_end):
        print('start', start)
        print('end', end)
        x1, y1 = start
        x2, y2 = end
        
        if x1 != x2:
            m = (y2- y1)/ (x2 - x1)
            c = y2 -m* x2    
            proj_x, proj_y = projection_point2line([0,0], m, c)
           
            d = dist_p2p([proj_x, proj_y], [0,0])
           
            if d != 0:
                
                angle = np.arctan2(proj_y, proj_x)
                world_walls.append([angle,d])
           
            if d == 0:
                angle = np.arctan(m) + 3*np.pi / 2 
                world_walls.append([angle,0])
        
        else:
            
            world_walls.append([np.pi, 0])
                
        
    return world_walls

wall_beginning = [[0, 0], [0, 0.000],[0.000, 20.0],[0, 23.700],[20.0 ,-1.73] ,[23.692, 0], [23.692, 7.244], [20.991, 23.692], [0.25999999999999784, 35.087], [0.25999999999999784, 35.087], [14.669999999999998, 35.087], [14.669999999999998, 28.482000000000003], [14.669999999999998, 27.959000000000003], [14.955999999999998, 27.959000000000003], [14.955999999999998, 24.224000000000004], [14.669999999999998, 24.224000000000004], [-2.0155849343766715e-15, 32.917], [-8.959000000000001, 35.087], [-8.959000000000001, 32.917]]
wall_end = [[0, 20.119], [23.692, 0], [-1.73, 20],[-1.73, 23.7],[23.7, -1.73],[23.692, 7.244], [20.991, 7.224], [20.991, 23.692], [14.669999999999998, 35.087], [-8.959000000000001, 35.087], [14.669999999999998, 34.146], [14.669999999999998, 27.959000000000003], [14.955999999999998, 27.959000000000003], [14.955999999999998, 24.224000000000004], [14.669999999999998, 24.224000000000004], [14.669999999999998, 23.570400000000003], [-1.5729363488248607e-15, 25.688000000000002], [-8.959000000000001, 32.917], [-2.0155849343766715e-15, 32.917]]

world_walls = wallcoords_to_parametric(wall_beginning, wall_end)
print(world_walls)