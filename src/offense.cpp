#include "offense.h"

using namespace std;
using namespace geometry_msgs; 
using namespace Eigen;

void playOffense(int robotNumber)
{
	if (robotNumber == 1)
	{
		// if the other team has control of the ball
		/*if (robot_has_ball(opp1, false))
		{

			wait_to_intercept(ally1, opp1, 1);

		}
		else if (robot_has_ball(opp2, false))
		{
			wait_to_intercept(ally1, opp2, 1);
	
		}
		else
		{
			play_getBehindBall(ally1, ball, 1);
		}
		//ROS_ERROR_STREAM("What do you think " << avg_distance_between_samples());
		//if (utility_vecLength(avg_distance_between_samples()) < .1)	//not moving
		//{
			//play_getBehindBall(ally1, ball, 1);
		//}
		//else			//Ball moving
		//{
			//play_getBehindBall(ally1, ball - (avg_distance_between_samples()*30), 1);
		//}*/

		//play_getBehindBall(ally1, ball, 1);
		//play_go_to_ball(ally1, ball, 1);
		/////skill_goToBall(ally1, ball, 1);
		/////check_collision(ally1, ball, 1);
		//play_getBehindBall(ally1, ball, 1);
		set_call_play_avoid(true);
		//play_rushGoal(ally2, ball, 2)
	}

	
	else if (robotNumber == 2)
	{
		set_robot(2);
		set_call_play_avoid(true);
		// if the other team has control of the ball
		/*if (robot_has_ball(opp1, false))
		{

			wait_to_intercept(ally2, opp1, 2);

		}
		else if (robot_has_ball(opp2, false))
		{
			wait_to_intercept(ally2, opp2, 2);
	
		}
		else
		{
			play_getBehindBall(ally2, ball, 2);
		}*/

		//play_getBehindBall(ally2, ball, 2);
		//play_rushGoal(ally2, ball, 2);
	}
}
