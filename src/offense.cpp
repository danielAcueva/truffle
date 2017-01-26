#include "offense.h"

using namespace std;
using namespace geometry_msgs; 
using namespace Eigen;

void playOffense(int robotNumber)
{
	if (robotNumber == 1){
		//play_rushGoal(ally1, ball, 1);
		play_getBehindBall(ally1, ball, 1);
		//skill_goToPoint(ally1, get_quadrant_center(2), 1);
	}
	else if (robotNumber == 2){
		//play_rushGoal(ally2, ball, 2);
		play_getBehindBall(ally1, ball, 1);
	}
}
