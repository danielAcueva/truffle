//Header guard
#ifndef O_INTERCEPT_AVOID_H
#define O_INTERCEPT_AVOID_H

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
#include "../skills.h"
#include "../helper.h"
#include "../strategy/strategy.h"

using namespace std;
using namespace geometry_msgs;
using namespace Eigen;

bool is_avoid_running();

void set_robot(int robot_number_in);

void set_call_play_avoid(bool input);

void intercept_avoid_tick();

//end of header guard
#endif