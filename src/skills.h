//Header guard
#ifndef SKILLS_H
#define SKILLS_H

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

void skill_followBallOnLine1(RobotPose robot, Vector2d ball, double x_pos, int robotId);

void skill_go_to_quadrant(RobotPose robot, Vector2d point, int robotId);

void skill_go_to_point(RobotPose robot, Vector2d point, int robotId);

bool skill_ball_align_quadrant(RobotPose robot, int quadrant, int robotId);

//end of header guard
#endif
