//Header guard
#ifndef O_INTERCEPT_BALL_H
#define O_INTERCEPT_BALL_H

#include <stdio.h>
#include <eigen3/Eigen/Eigen>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include "../globals.h"
#include "../controller1.h"
#include "../math.h"

using namespace std;
using namespace geometry_msgs;
using namespace Eigen;



//end of header guard
#endif
void wait_to_intercept(RobotPose robot, RobotPose opp, int robotId);
