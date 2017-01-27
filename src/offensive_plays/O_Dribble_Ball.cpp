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
int current_quadrant = 0;

void set_call_play_dribble(bool input)
{
	call_play_dribble = input;
}

void dribble_ball_tick()
{
	switch(DB_state)	//State Transitions
	{
		case wait_for_play:
		{
			if (call_play_dribble)			//Check to see if the play has been called
			{
				DB_state = find_target_quadrant;	//The play has been called. Start the SM
			}
			break;
		}
		case find_target_quadrant:
		{
			DB_state = align_with_ball;
			break;
		}
		case align_with_ball:
		{
			break;
		}
		case move_to_quadrant:
		{
			break;
		}
		case arrived:
		{
			break;
		}
	}
	switch(DB_state)	//State Actions
	{
		case wait_for_play:
		{
			break;
		}
		case find_target_quadrant:
		{
			target_quadrant = find_open_quadrant(ally1.pos);	//Find the best quadrant to move to
			break;
		}
		case align_with_ball:
		{
			//Write code to align with ball. return true if aligned
			break;
		}
		case move_to_quadrant:
		{
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