#include "O_Intercept_Ball.h"

#include "../quadrant.h"
#include "../strategy/strategy.h"
using namespace std;
using namespace geometry_msgs; 
using namespace Eigen;

//#define PI 3.14159265


    /*   Col0 . Col1 . Col2 . Col3
    *   ______.______.______._______
    *  |      .      .      .      |
    *  |  Q0  .  Q1  .  Q2  .  Q3  |   Row0
    *  |...........................|.....
    * _|      .      .      .      |_
    *|    Q4  .  Q5  .  Q6  .  Q7    | Row1
    *|...............................|...
    *|        .      .      .        |
    *|_   Q8  .  Q9  .  Q10 .  Q11  _| Row2
    *  |...........................|.....
    *  |      .      .      .      |  
    *  |  Q12 .  Q13 .  Q14 .  Q15 |   Row3
    *  |______.______.______.______|   
    *
    */

/**
*
* opp robot that has control of the ball
*/
void wait_to_intercept(RobotPose robot, RobotPose opp, int robotId)
{
	// opp has control of ball, so position robot between opp and goal

	int ball_quad_row = get_quadrant_row(ball);
	int ball_quad_col = get_quadrant_column(ball);




	// if ball is to the right of robot
    if (ball(0) > robot.pos(0))
    {        //if the robot is in front of the ball

		// move one quadrant left of the ball
		if (ball_quad_col != 0)
		{
			ball_quad_col--;
		}
		else
		{
			// need to still move left, just not a whole quad
		}
	}
	else
	{
		// move one quadrant left of the ball
		if (ball_quad_col != 3)
		{
			ball_quad_col++;
		}
		else
		{
			// still move right, just not a whole quad
		}
	}

	int ball_quad = get_quadrant(ball_quad_col, ball_quad_row);



	Vector2d point = get_quadrant_center(ball_quad);



        if (robot.pos(1) < ball(1)){    //The robot is in Q1
            point(1) -= .2;             //Go to a point above the ball
        }
        else{                           //The robot is in Q4
            point(1) += .2;             //Go to a point below the ball
        }
    

	skill_goToPoint(robot, point, robotId);

	return;
}