//Header guard
#ifndef OFFENSE_H
#define OFFENSE_H

#include <stdio.h>
#include <eigen3/Eigen/Eigen>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include "globals.h"
#include "controller1.h"
#include "math.h"
#include "strategy/strategy.h"
#include "quadrant.h"
#include "offensive_plays/O_Avoid_Corner.h"
#include "offensive_plays/O_Dribble_Ball.h"
#include "offensive_plays/O_Intercept_Ball.h"
#include "offensive_plays/O_Intercept_Avoid.h"
#include "helper.h"

using namespace std;
using namespace geometry_msgs;
using namespace Eigen;

void playOffense(int robotNumber);

//end of header guard
#endif
