#include "defense.h"

using namespace std;
using namespace geometry_msgs; 
using namespace Eigen;

void playDefense(int robotNumber)
{
	if (robotNumber == 1){
		skill_followBallOnLine(ally1, ball, -2 * FIELD_WIDTH / 6, 1);
	}
	else if (robotNumber == 2){
		skill_followBallOnLine(ally2, ball, -2 * FIELD_WIDTH / 6, 2);
	}
}
