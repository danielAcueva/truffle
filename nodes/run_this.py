#!/usr/bin/env python

import struct
import time
import serial
import matplotlib.pyplot as plt
import motion
import rospy
from std_msgs.msg import String


totalTime = 3   #seconds
sampleRate = 50 #samples per second
pulsePerRotation = 116.2 #New motors

#initialize arrays
times = []
speedsM1 = []
speedsM2 = []
speedsM3 = []

speedM1 = 0.0 # rot/s
speedM2 = 0.0 # rot/s
speedM3 = 0.0 # rot/s

#ser = serial.Serial('COM4', 115200, timeout=None) #windows
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=None) #linux, (read note on webpage about ttyAMA0 first)

def _handle_velocity(Twist):
    global _velocity
    _velocity = Twist

def main():
    rospy.init_node('run_this', anonymous=False)

    rospy.Subscriber('velocity', _velocity, _handle_velocity)
    
    # Set the PIDQ values for all motors
    #setPID(0, 1, 1, 1100)
    setPID(1, 1, 1, 450)
    setPID(2, 1, 1, 350)
    setPID(3, 1, 1, 450)

    # Set tick period (triggers PID control) and velocity filter corner frequency
    setT(20, 50)


    vx = _velocity.linear.x
    vy = _velocity.linear.y
    vw = _velocity.angular.z
    #vy = Pose2D.item(2)
    #vw = Pose2D.item(3)

    setVelocity(vx,vy,vw,0)

    rospy.spin()
    disengage()

# Note: If you would like to port the readFloat and writeFloat functions to C++, simply use the functions provided 
# in the PSoC implementation: https://github.com/embeddedprogrammer/psoc-pid-controller/blob/master/serial.c
def writeFloat(f):
    ser.write(struct.pack('>i', int(f*1000)))
def readFloat():
    return float(struct.unpack('>i', ser.read(4))[0])/1000
def setPower(p1, p2, p3):
    ser.write('p')
    writeFloat(p1)
    writeFloat(p2)
    writeFloat(p3)
def setSpeed(s1, s2, s3):
    ser.write('s')
    writeFloat(s1)
    writeFloat(s2)
    writeFloat(s3)
def setPID(motor, p, i, qpps): #use motor = 0 to set all motors
    ser.write('k')
    ser.write(str(motor))
    writeFloat(p)
    writeFloat(i)
    writeFloat(qpps)
def setT(period_ms, tau_ms):
    ser.write('t')
    writeFloat(period_ms)
    writeFloat(tau_ms)
def getSpeed():
    ser.write('v')
    return readFloat(), readFloat(), readFloat()
def getEncoderCount():
    ser.write('e')
    return readFloat(), readFloat(), readFloat()
def disengage():
    ser.write('d') 

def setVelocities(vx, vy, omega, theta):
    robot_frame = motion.WorldToRobotBody(vx, vy, omega, theta)
    vbx = robot_frame.item(0) # desired x velocity in robot body frame
    vby = robot_frame.item(1)
    vbw = robot_frame.item(2) # desired omega
    speedMotors = motion.getWheelSpeed(vbx, vby, vbw) # this should spin the robot
    speedM1 = speedMotors.item(0) # rot/s
    speedM2 = speedMotors.item(1) # rot/s
    speedM3 = speedMotors.item(2) # rot/s
    setSpeed(speedM1*pulsePerRotation, speedM2*pulsePerRotation, speedM3*pulsePerRotation)




