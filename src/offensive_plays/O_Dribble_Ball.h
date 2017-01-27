//Header guard
#ifndef O_DRIBBLE_BALL_H
#define O_DRIBBLE_BALL_H

#include <stdio.h>
#include <eigen3/Eigen/Eigen>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include "../globals.h"
#include "../controller1.h"
#include "../math.h"
#include "../quadrant.h"

using namespace std;
using namespace geometry_msgs;
using namespace Eigen;

//Call the play on or off
void set_call_play_dribble(bool input);

//State machine tick function
void dribble_ball_tick();

//end of header guard
#endif