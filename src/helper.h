//Header guard
#ifndef HELPER_H
#define HELPER_H

#include <stdio.h>
#include <eigen3/Eigen/Eigen>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include "globals.h"
#include "controller1.h"
#include "math.h"
#include "quadrant.h"

using namespace std;
using namespace geometry_msgs;
using namespace Eigen;

//This function returns whether the specified robot has the ball
//If the ball is close to the robot, and between the robot and
//the goal, the robot has the ball
bool robot_has_ball(RobotPose robot, bool myTeam);

bool robot_in_quadrant_middle(RobotPose robot, int quadrant);

bool ball_in_area(RobotPose robot);

//end of header guard
#endif
