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

bool robot_in_quadrant_middle(RobotPose robot, int quadrant)
{
	Vector2d position = robot.pos;
	Vector2d center_quadrant = get_quadrant_center(quadrant);
	if(utility_vecLength(center_quadrant - position) < .1)
	{
		return true;
	}
	return false;
}

//This function lets us know if the ball is still next to the robot.
//This is for telling if the ball is close, but not necessarily in possesion
//For dribbling
bool ball_in_area(RobotPose robot)
{
	Vector2d robotPosition = robot.pos;				//Get the robot's 2d XY coordinates
	//Check if the ball is 10cm in front of us
	if(utility_vecLength(robotPosition - ball) < .2)
	{
		return true;						//We have the Ball!
	}
	return false;
}