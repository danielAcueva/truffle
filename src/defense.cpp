#include "defense.h"

using namespace std;
using namespace geometry_msgs; 
using namespace Eigen;

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

	double theta = -atan((ball(1) - goal(1))/(ball(0) - 0));


	double x_pos = (-1.0 * cos(theta));

	if (robotNumber == 1)
		skill_followBallOnLine(ally1, ball, -1, 1);
	else if (robotNumber == 2)
		skill_followBallOnLine(ally2, ball, -1, 2);
}
