#!/usr/bin/env python
import numpy as np

#radius of wheel - update later
r_wheel = 1.125

#radius from center to center of wheel
r1 = 2.7 #inches
r2 = 2.7
r3 = 2.7

# robot with wheel placement
#         2
#       _____
#      /     \
#     |       |
#    1\       /3
#      \     /
#       \___/
#           
#         

#position of the center of the ith wheel expressed with respect
#to a reference frame fixed in the body of the robot
#rb = (rbx, rby, 0)T
r_th1 = (18.9)*(np.pi/180)   #angle from origin to wheel
r_x1  = np.cos(r_th1)*r1 #distance from y-axis to wheel
r_y1  = np.sin(r_th1)*r1 #distance from x-axis to wheel
#r_th2 = (161.03)*(np.pi/180)
r_th2 = (270.0)*(np.pi/180)
r_x2  = np.cos(r_th2)*r2 
r_y2  = np.sin(r_th2)*r2 
#r_th3 = (270.0)*(np.pi/180)
r_th3 = (161.03)*(np.pi/180)
r_x3  = np.cos(r_th3)*r3 
r_y3  = np.sin(r_th3)*r3 



#unit vectors that point in the direction of spin of the ith wheel
#rolling direction of wheel
#sb = (sbx, sby, 0)T
s_th1 = r_th1 + np.pi/2 #angle of the wheel off of the r vector ( should be orthogonal )
s_x1  = np.cos(s_th1)
s_y1  = np.sin(s_th1)
s_th2 = r_th2 + np.pi/2
s_x2  = np.cos(s_th2)
s_y2  = np.sin(s_th2)
s_th3 = r_th3 + np.pi/2 
s_x3  = np.cos(s_th3)
s_y3  = np.sin(s_th3)

#wheel has radius R and angular speed omega
M = np.matrix([[s_x1, s_y1,(s_y1*r_x1 - s_x1*r_y1)],
               [s_x2, s_y2,(s_y2*r_x2 - s_x2*r_y2)],
               [s_x3, s_y3,(s_y3*r_x3 - s_x3*r_y3)]])
M = M/r_wheel

print M

