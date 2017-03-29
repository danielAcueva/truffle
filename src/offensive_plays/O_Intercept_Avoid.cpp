#include "O_Intercept_Avoid.h"

using namespace std;
using namespace geometry_msgs; 
using namespace Eigen;

//FIXED TO ROBOT ALLY1. CHANGE AT SOME POINT!!!!!!!!
#define STATE_EXPIRE 10

//Create an enumerated type for all of the states in the SM
//The starting point will be wait_for_play
enum IA_states { wait_for_play, check_avoid_point, go_to_avoid_point, 
			go_to_ball, arrived} IA_state = wait_for_play; 

int state_count = 0;
Vector2d avoid_point;
bool call_play_avoid = false;
bool avoid_running = false;
int robot_number = 1;
RobotPose robot = ally1;

bool is_avoid_running()
{
	return avoid_running;
}

void set_call_play_avoid(bool input)
{
	call_play_avoid = input;
}

void set_robot(int robot_number_in)
{
	if (robot_number_in == 1)
	{
		robot = ally1;
		robot_number = 1;
	}
	else
	{
		robot = ally2;
		robot_number = 2;
	}
}

void intercept_avoid_tick()
{
	switch(IA_state)	//State Actions
	{
		case wait_for_play:
		{
			break;
		}
		case check_avoid_point:
		{
			state_count = 0; 			//Reset the state count
			break;
		}
		case go_to_avoid_point:
		{
			//cout << "curr point: " << ally1.pos(0) << ", " << ally1.pos(1) << endl;
			//cout << "ally point: " << ally2.pos(0) << ", " << ally2.pos(1) << endl;

			//cout << "new point: " << avoid_point(0) << ", " << avoid_point(1) << endl;

			skill_goToPoint(robot, avoid_point, robot_number);
			state_count++;
			break;
		}
		case go_to_ball:
		{
			play_getBehindBall(robot, ball, robot_number);
			state_count++;
			break;
		}
		case arrived:
		{
			break;
		}
	}

	switch(IA_state)	//State Transitions
	{
		case wait_for_play:
		{
			if (call_play_avoid)
			{
				IA_state = check_avoid_point;
			}
			break;
		}
		case check_avoid_point:
		{
			//Check if there is an abject in my way
			if (object_in_path(robot, robot_number))
			{
				//Set the point so we can move toward it 
				avoid_point = check_collision(robot, ball, robot_number);
				//Go to the avoid point state
				state_count = 0;
				IA_state = go_to_avoid_point;
			}
			else
			{
				state_count = 0;
				//Nothing in the way, go to the ball
				IA_state = go_to_ball;
			}
			break;
		}
		case go_to_avoid_point:
		{
			if (state_count == STATE_EXPIRE)
			{
				IA_state = check_avoid_point;
			}
			break;
		}
		case go_to_ball:
		{
			if (state_count == STATE_EXPIRE)
			{
				IA_state = check_avoid_point;
			}
			break;
		}
		case arrived:
		{
			break;
		}
	}

}