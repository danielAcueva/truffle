#include "offense.h"

using namespace std;
using namespace geometry_msgs; 
using namespace Eigen;

void playOffense(int robotNumber)
{
	if (robotNumber == 1){
		set_call_play_dribble(true);
		/*if (robot_has_ball(ally1, true))
		{
			set_call_play_dribble(true);
		}
		else
		{
			//play_getBehindBall(ally1, ball, 1);
			set_call_play_dribble(false);
		}
		if (!is_dribble_running())
		{
			//play_getBehindBall(ally1, ball, 1);
		}*/
		//play_rushGoal(ally1, ball, 1);
		////play_getBehindBall(ally1, ball, 1);
		//skill_goToPoint(ally1, get_quadrant_center(9), 1);
		//bool mybool = robot_has_ball(ally1, true, 1);
	}
	else if (robotNumber == 2){
		//play_rushGoal(ally2, ball, 2);
		play_getBehindBall(ally1, ball, 1);
	}
}
