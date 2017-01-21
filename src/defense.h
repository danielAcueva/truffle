//Header guard
#ifndef DEFENSE_H
#define DEFENSE_H

#include <stdio.h>
#include <eigen3/Eigen/Eigen>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include "globals.h"
#include "controller1.h"
#include "math.h"
#include "strategy.h"

using namespace std;
using namespace geometry_msgs;
using namespace Eigen;

void playDefense(int robotNumber);

void playArchDefense(int robotNumber);


//end of header guard
#endif
