//Header guard
#ifndef TEST_H
#define TEST_H

#include <stdio.h>
#include <eigen3/Eigen/Eigen>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include "globals.h"
#include "controller1.h"
#include "math.h"

using namespace std;
using namespace geometry_msgs;
using namespace Eigen;

//Functions
//TEST. Follow the ball!
void test_followBall(RobotPose robot, Vector2d ball, int robotId);

//end of header guard
#endif