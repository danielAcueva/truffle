//Header guard
#ifndef MATH_H
#define MATH_H

#include <stdio.h>
#include <eigen3/Eigen/Eigen>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include "globals.h"
#include "controller1.h"

using namespace std;
using namespace geometry_msgs;
using namespace Eigen;

//MATH FUNCTIONS
int utility_sgn(double val);

// utility - saturate velocity
//  Saturate the commanded velocity .
Vector3d utility_saturateVelocity(Vector3d v);

Vector3d utility_saturateVelocityHard(Vector3d v);

double utility_vecLength(Vector2d v);

Vector2d utility_unitVector(Vector2d v);

double utility_angleMod(double angle);

//end of header guard
#endif
