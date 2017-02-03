#include "defense.h"

using namespace std;
using namespace geometry_msgs; 
using namespace Eigen;

//#define PI 3.14159265

void playDefense(int robotNumber)
{


	if(robotNumber == 1)
	{
		if( ( ((ally1.pos(0) - ball(0)) > -0.2) ) && (abs(ball(1)- ally1.pos(1)) < 0.1))
		{
			play_getBehindBall(ally1, ball, 1);
		}
		else
		{
			if(ball(0) > 0)
				playMovingScreen(1);
			else
				playTriDefense(1);
		}
	}
	else
	{
		if( ( ((ally2.pos(0) - ball(0)) > -0.2) ) && (abs(ball(1)- ally2.pos(1)) < 0.1))
		{
			play_getBehindBall(ally2, ball, 2);
		}
		else
		{
			if(ball(0) > 0)
				playMovingScreen(2);
			else
				playTriDefense(2);
		}
	}
}




void playMovingScreen(int robotNumber)
{
	// Midpoint Calulcation
	// Midpoint between oponent and ball
	// assumes opponent1 is the offensive opponent

	double x_mid = (ball(0) + opp1.pos(0))/2;
	double y_mid = (ball(1) + opp1.pos(1))/2;

	Vector2d p;

	p << x_mid, y_mid;
	
	if (robotNumber == 1)
		skill_goToPoint(ally1,p,1);
	else if (robotNumber == 2)
		skill_goToPoint(ally2,p,2);
}

void playTriDefense(int robotNumber)
{

	//Triangle Calculations

	double abs_ball = abs(ball(1));

	double x_posTri = (abs_ball + 2) / -1.5; // y = mx + b
											// x = (y - b)/m

	if (robotNumber == 1)
		skill_followBallOnLine(ally1, ball, x_posTri, 1);
	else if (robotNumber == 2)
		skill_followBallOnLine(ally2, ball, x_posTri, 2);
}






void playArchDefense(int robotNumber)
{
	//Arch Calculations

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

void playSpin(int robotNumber)
{
	Vector3d v;

    v(0) = 0;
    v(1) = 0;
    v(2) = 5.9*PI;

    v = utility_saturateVelocity(v);
    publish_moveRobot(v, robotNumber);

}