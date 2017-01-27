#include "defense.h"

using namespace std;
using namespace geometry_msgs; 
using namespace Eigen;

#define PI 3.14159265

void playDefense(int robotNumber)
{


	
	if (robotNumber == 1)
		playArchDefense(1);
	else if (robotNumber == 2)
		playArchDefense(2);
}

void playArchDefense(int robotNumber)
{
	//Arch Calculations
	//double own_goal = 0;

	//own_goal = goal(0) - FIELD_WIDTH;

	double ballx_pos = ball(0) + (FIELD_WIDTH/2);

	double theta = 4.0 * atan(ball(1)/ballx_pos);

	double temp = sin(theta + (PI/2));
	if(temp < 0)
		temp = -temp;

	double x_posArch = temp - (FIELD_WIDTH/2) - 0.5;

	if (robotNumber == 1)
		skill_followBallOnLine(ally1, ball, x_posArch, 1);
	else if (robotNumber == 2)
		skill_followBallOnLine(ally2, ball, x_posArch, 2);
}
