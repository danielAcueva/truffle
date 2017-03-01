#!/usr/bin/env python
import numpy as np
import param as P


#linear velocity of the center of the body fixed reference frame
#vb = (vbx, vby, 0)T
#V = (R*T)*Inv(M)*omega

#angular velocity of the robot
#w = (0, 0, w)T

# wheel speed

def getWheelSpeed(v_x, v_y, omega_body):
    v_desired = np.matrix([[v_x],
                           [v_y],
                           [omega_body]])
    omega_wheel = P.M*v_desired
    
    return omega_wheel

def WorldToRobotBody(v_xw,v_yw,omega_world,theta):
    vw_desired = np.matrix([[v_xw],
                            [v_yw],
                            [omega_world]])
    vb_desired = R(theta)*vw_desired
    return vb_desired    
    

def R(theta):
            #rotation matrix
    ctheta = np.cos(theta)
    stheta = np.sin(theta)
    L = np.matrix([[ ctheta, stheta, 0.0],
                   [-stheta, ctheta, 0.0],
                   [    0.0,    0.0, 1.0]])

    return L
