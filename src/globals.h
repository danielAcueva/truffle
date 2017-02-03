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
#include <queue>

using namespace std;
using namespace geometry_msgs;
using namespace Eigen;

//Controller constants
#define PI 3.14159
#define ROBOT_MAX_VXY 2.0
#define ROBOT_MAX_OMEGA 6*PI //Was 2*M_PI
#define CONTROL_K_XY 5
#define CONTROL_K_OMEGA 2
#define FIELD_WIDTH 3.40  // in meters
#define FIELD_HEIGHT 2.38 
#define ROBOT_RADIUS 0.10

//use the standard queue
//This standard queue is infinite
//start by filling the queue to a size with 0's
//Create a function that checks if it is a certain size
//if it is pop and push; (like an overwrite push)
extern queue<Vector2d> stored_ball_values;// = new queue<Vector2d>();
//Create another function that prints the queue
//Just copy the queue and pop each element. Store in 

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

extern Vector2d ball_values[10];

void reset_ball_values();

void push_ball_values(Vector2d data);

Vector2d *return_ball_values();

Vector2d avg_distance_between_samples();

//Functions

//end of header guard
#endif
