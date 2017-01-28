#include "offense.h"

using namespace std;
using namespace geometry_msgs; 
using namespace Eigen;

void playOffense(int robotNumber)
{
	if (robotNumber == 1)
	{
		// if the other team has control of the ball
		if (robot_has_ball(opp1, false))
		{

			play_test(ally1, opp1, 1);

		}
		else if (robot_has_ball(opp2, false))
		{
			play_test(ally1, opp2, 1);
	
		}
		else
		{
			play_getBehindBall(ally1, ball, 1);
		}
	}

	
	else if (robotNumber == 2)
	{
		// if the other team has control of the ball
		if (robot_has_ball(opp1, false))
		{

			play_test(ally1, opp1, 1);

		}
		else if (robot_has_ball(opp2, false))
		{
			play_test(ally1, opp2, 1);
	
		}
		else
		{
			play_getBehindBall(ally1, ball, 1);
		}
	}
}
