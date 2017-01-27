//Header guard
#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdio.h>
#include <eigen3/Eigen/Eigen>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include "soccerref/GameState.h"

using namespace std;
using namespace geometry_msgs;
using namespace Eigen;

//Controller constants
#define ROBOT_MAX_VXY 2.0
#define ROBOT_MAX_OMEGA 6*M_PI //Was 2*M_PI
#define CONTROL_K_XY 5
#define CONTROL_K_OMEGA 2
#define FIELD_WIDTH 3.40  // in meters
#define FIELD_HEIGHT 2.38 
#define ROBOT_RADIUS 0.10

//Structs
struct RobotPose
{
    Vector2d pos;
    double theta;
};

extern string team;
extern Vector2d goal;	

extern RobotPose ally1, ally2;						//Ally's are made of struct pose
extern RobotPose opp1, opp2;						//Opponents are made of struct pose
extern Vector2d ball;								//Ball is a 2d vector
extern Vector2d ally1_startingPos;					//starting position. 2d vector
extern Vector2d ally2_startingPos;					//starting position. 2d vector


//Functions

//end of header guard
#endif
