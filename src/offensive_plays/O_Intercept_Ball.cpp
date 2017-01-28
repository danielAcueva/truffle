#include "O_Intercept_Ball.h"

#include "../quadrant.h"
#include "../strategy/strategy.h"
using namespace std;
using namespace geometry_msgs; 
using namespace Eigen;

#define PI 3.14159265


/**
*
* opp robot that has control of the ball
*/
void play_test(RobotPose robot, RobotPose opp, int robotId)
{
	// opp has control of ball, so position robot between opp and goal


	//double x_pos = opp.pos(0) - 0.8;

		//skill_followBallOnLine(ally1, ball, x_pos, robotId);

		// or quad based

	int ball_quad = get_quadrant(ball);

	if (ball_quad != 0 && ball_quad != 8 && ball_quad != 12)
	{
		ball_quad--;
	}

	// move one quadrant left of the ball
	Vector2d move_pos = get_quadrant_center(ball_quad);

   skill_goToPoint(robot, move_pos, robotId);




    return;

}