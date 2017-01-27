#include "helper.h"

using namespace std;
using namespace geometry_msgs; 
using namespace Eigen;

//////////////////////////////////////////////////////////////////////////

//This function returns whether the specified robot has the ball
//If the ball is close to the robot, and between the robot and
//the goal, the robot has the ball
bool robot_has_ball(RobotPose robot, bool myTeam)
{
	Vector2d robotPosition = robot.pos;				//Get the robot's 2d XY coordinates
	if (myTeam)										//if the robot is on our team
	{
		if (ball(0) > robotPosition(0))				//the ball is in front of you
		{
			//Check if the ball is 10cm in front of us
			if(utility_vecLength(robotPosition - ball) < .2)
			{
				return true;						//We have the Ball!
				cout << "we have the ball" << endl;
			}

		}
		else										//The ball is behind us, we dont have it
		{
			return false;							//We don't have the ball
			cout << "we DONT have the ball" << endl;
		}
	}
	else											//The robot is not on our team
	{
		if (ball(0) < robotPosition(0))				//the ball is in front of the enemy
		{
			//Check if the ball is 10cm in front of us
			if(utility_vecLength(robotPosition - ball) < .2)
			{
				return true;						//they have the ball
			}

		}
		else										//The ball is behind them
		{
			return false;							//they don't have the ball
		}		
	}
	return false;
	cout << "we DONT have the ball" << endl;
}