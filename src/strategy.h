//Header guard
#ifndef STRATEGY_H
#define STRATEGY_H

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

//Strategy Functions
// skill - follow ball on line
//   Follows the y-position of the ball, while maintaining x-position at x_pos. 
//   Angle always faces the goal.
void skill_followBallOnLine(RobotPose robot, Vector2d ball, double x_pos, int robotId);

// skill - go to point
//   Travels towards a point. Angle always faces the goal.
void skill_goToPoint(RobotPose robot, Vector2d point, int robotId);

// play - rush goal
//   - go to position behind ball
//   - if ball is between robot and goal, go to goal
// NOTE:  This is a play because it is built on skills, and not control
// commands.  Skills are built on control commands.  A strategy would employ
// plays at a lower level.  For example, switching between offense and
// defense would be a strategy.
void play_rushGoal(RobotPose robot, Vector2d ball, int robotId);

void play_getBehindBall(RobotPose robot, Vector2d ball, int robotId);



//end of header guard
#endif
