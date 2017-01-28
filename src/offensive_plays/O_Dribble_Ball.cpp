#include "O_Dribble_Ball.h"

using namespace std;
using namespace geometry_msgs; 
using namespace Eigen;

//FIXED TO ROBOT ALLY1. CHANGE AT SOME POINT!!!!!!!!

//Create an enumerated type for all of the states in the SM
//The starting point will be wait_for_play
enum DB_States { wait_for_play, find_target_quadrant, align_with_ball, 
			move_to_quadrant, arrived} DB_state = wait_for_play; 

bool call_play_dribble = false;
int target_quadrant = 0;
bool dribble_running = false;

bool is_dribble_running()
{
	return dribble_running;
}

void set_call_play_dribble(bool input)
{
	call_play_dribble = input;
}

bool in_shooting_range()
{
	int current_quadrant_ball = get_quadrant(ball);
	if ((current_quadrant_ball == 7) ||
		(current_quadrant_ball == 11))
	{
		return true;
	}
	else
	{
		return false;
	}
}

void dribble_ball_tick()
{
	switch(DB_state)	//State Transitions
	{
		case wait_for_play:
		{
			if (call_play_dribble)			//Check to see if the play has been called
			{
				dribble_running = true;
				DB_state = find_target_quadrant;	//The play has been called. Start the SM
			}
			break;
		}
		case find_target_quadrant:
		{
			//Check if we are in shooting range
			if (in_shooting_range())
			{
				DB_state = wait_for_play;		//In shooting range, no need to dribble
			}
			else								
			{
				DB_state = align_with_ball;		//Not in shooting range. Continue
			}
			break;
		}
		case align_with_ball:
		{
			//Check to see if we have aligned with the ball and quadrant
			if (skill_ball_align_quadrant(ally1, target_quadrant, 1))
			{
				DB_state = move_to_quadrant;			//transition to move to quadrant
			}
			break;
		}
		case move_to_quadrant:
		{
			if (robot_in_quadrant_middle(ally1, target_quadrant))
			{
				DB_state = arrived;
			}
			break;
		}
		case arrived:
		{
			if (ball_in_area(ally1))							//We still have the ball
			{
				DB_state = find_target_quadrant;		//Keep going, this is working
			}
			else
			{
				DB_state = wait_for_play;
			}
			break;
		}
	}
	switch(DB_state)	//State Actions
	{
		case wait_for_play:
		{
			dribble_running = false;
			break;
		}
		case find_target_quadrant:
		{
			//target_quadrant = find_open_quadrant(ally1.pos);	//Find the best quadrant to move to
			target_quadrant = find_open_quadrant(ball);
			break;
		}
		case align_with_ball:
		{
			//Write code to align with ball. return true if aligned
			break;
		}
		case move_to_quadrant:
		{
			skill_go_to_point(ally1, get_quadrant_center(target_quadrant), 1);
			//Write a function that computs a vector from robot to quad center
			//If within some close ammount, call him there, return true
			break;
		}
		case arrived:
		{
			//Write a function that checks if the ball is still close to me. 
			//Do this by computing the vector, and checking distance
			//Return true if we still have it. false if not.
			break;
		}
	}
}