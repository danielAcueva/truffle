#include "strategy.h"
#include "../quadrant.h"

using namespace std;
using namespace geometry_msgs; 
using namespace Eigen;

//Strategy Functions
// skill - follow ball on line
//   Follows the y-position of the ball, while maintaining x-position at x_pos. 
//   Angle always faces the goal.
void skill_followBallOnLine(RobotPose robot, Vector2d ball, double x_pos, int robotId)
{
    // control x position to stay on current line
    //CONTROL_K_XY is constant value of 5. multiplied by offset.
    //x pos is -2 * field width. 
    double vx = CONTROL_K_XY * (x_pos - robot.pos(0));

    // control y position to match the ball's y-position
    //CONTROL times offset between ball(y) and robot Y position
    double vy = CONTROL_K_XY * (ball(1) - robot.pos(1));

    if (abs(ball(1)) > 0.75) // if playing defense_arch, keeps out of corners
    {
        if(ball(1) < 0)
            vy = CONTROL_K_XY * (-0.5 - robot.pos(1)); 
        else
            vy = CONTROL_K_XY * (0.5 - robot.pos(1));
    }
    else
        vy = CONTROL_K_XY * (ball(1) - robot.pos(1));

    

    // control angle to face the goal
    Vector2d dirGoal = goal - robot.pos;						//Offset from robot to goal
    double theta_d = atan2(dirGoal(1), dirGoal(0));				//Theta is the arc tan of the goal
																//This ensures that the robot faces the goal
																//Arctan (y, x)
	//robot is of type robotpose. this is a struct with 2d coordinates, and theta
	//CONTROL_K_OMEGA is constant value 2
	//this negative value is multiplied by theta difference between robot and calculated
    double omega = -CONTROL_K_OMEGA * (robot.theta - theta_d); 
    
    // Output velocities to motors  
    Vector3d v;							//Create a 3d vector. XY and Z
    v << vx, vy, omega;					//Fill vector with calculated values
    v = utility_saturateVelocity(v);	//MIGHT BE USEFUL. I DONT KNOW WHAT IT DOES
    publish_moveRobot(v, robotId);				//Move the robot the calculated value
}

// skill - go to point
//   Travels towards a point. Angle always faces the goal.
void skill_goToPoint(RobotPose robot, Vector2d point, int robotId)
{
    Vector2d dirPoint = point - robot.pos;
    Vector2d vxy = dirPoint * CONTROL_K_XY;

    // control angle to face the goal
    Vector2d dirGoal = goal - robot.pos;
    double theta_d = atan2(dirGoal(1), dirGoal(0));
    double omega = -CONTROL_K_OMEGA * (robot.theta - theta_d); 

    // Output velocities to motors
    Vector3d v;
    v << vxy, omega;
    v = utility_saturateVelocity(v);
    publish_moveRobot(v, robotId);
}

// play - rush goal
//   - go to position behind ball
//   - if ball is between robot and goal, go to goal
// NOTE:  This is a play because it is built on skills, and not control
// commands.  Skills are built on control commands.  A strategy would employ
// plays at a lower level.  For example, switching between offense and
// defense would be a strategy.
void play_rushGoal(RobotPose robot, Vector2d ball, int robotId)
{
    // normal vector from ball to goal
    //Send the difference between goal position and ball position
    //Normalized vector returned
    Vector2d n = utility_unitVector(goal - ball);

    // compute position 10cm behind ball, but aligned with goal.
    // Ball position - 10 cm (nomalized vector aligning the ball and goal)
    //This should put the robot behind the ball
    Vector2d position = ball - 0.2*n;

    //Keep going towards goal
    if(utility_vecLength(position - robot.pos) < 0.21){
        skill_goToPoint(robot, goal, robotId);
    }
    else{
        skill_goToPoint(robot, position, robotId);
    }
}

// skill - go to point
//   Travels towards a point. Angle always faces the goal.
void skill_goToBall(RobotPose robot, Vector2d point, int robotId)
{
    //Find the unit vector from point to robot difference
    Vector2d pointDiff = utility_unitVector(point - robot.pos);
    Vector2d vxy;                                       //Create a vector
    if ((utility_vecLength(robot.pos - ball)) > .6){    //When we are far from ball
        vxy = pointDiff;// * CONTROL_K_XY;                 //Go to max velocity
    }
    else if ((utility_vecLength(robot.pos - ball)) < .1){   //When we are close to the ball
        vxy = pointDiff;// * CONTROL_K_XY;                     //Go to a slower velocity
    }
    else{
        vxy = pointDiff;                                //Go to a slower velocity       
    }
    // control angle to face the goal
    Vector2d dirGoal = goal - robot.pos;
    double theta_d = atan2(dirGoal(1), dirGoal(0));
    double omega = -CONTROL_K_OMEGA * (robot.theta - theta_d); 

    // Output velocities to motors
    Vector3d v;
    v << vxy, omega;
    v = utility_saturateVelocity(v);
    publish_moveRobot(v, robotId);
}

Vector2d avoid_robots(Vector2d point)
{
    RobotPose opp_quad;
    int quad = get_quadrant(point);
    int row = get_quad_row(quad);
    int col = get_quad_col(quad);

    // check if any robots are in the way of point
    if (get_robot_in_quad(quad, &opp_quad))
    {
        if (row == 1)
        {
            point(1) -= .5; 
            /*
            // change pos to row 2 if possible
            if (is_robot_in_quad(quad, opp_quad) != 2)
            {
                row = 2;
            }
            else
            {
                row = 3;
            }
            */
        }

        else if (row == 2)
        {
                point(1) += .5; 

            /*
            // change pos to row 2 if possible
            if (is_robot_in_quad(quad, opp_quad) != 1)
            {
                row = 1;
            }
            else
            {
                row = 3;
            }  
             */  
        }
        else if (row == 3)
        {
            point(1) += .5; 

            /*    
            // change pos to row 2 if possible
            if (is_robot_in_quad(quad, opp_quad) != 2)
            {
                row = 2;
            }
            else
            {
                row = 1;
            }   
             */ 
        }  
        else
        {
                point(1) -= .5; 


            /*
            // change pos to row 2 if possible
            if (is_robot_in_quad(quad, opp_quad) != 1)
            {
                row = 1;
            }
            else
            {
                row = 2;
            }  
             */  
        }

       // quad = get_quadrant(col, row);
        //return get_quadrant_center(quad);
    }

    return point;

}

void play_getBehindBall(RobotPose robot, Vector2d ball, int robotId)
{
    /*
    *   ___________________________
    *  |             :             |
    * _|     Q2      :     Q1      |_
    *|...............*...............|
    *|_              :^Ball         _|
    *  |     Q3      :     Q4      |
    *  |_____________:_____________|
    */
      
    Vector2d point;                     //Create a point
    if (robot.pos(0) > ball(0)){        //if the robot is in front of the ball
                                        //the robot is in Q1 or Q4 
        if (robot.pos(1) < ball(1)){    //The robot is in Q1
            point = ball;
            point(1) -= .2;             //Go to a point above the ball
        }
        else{                           //The robot is in Q4
            point = ball;
            point(1) += .2;             //Go to a point below the ball
        }
    }
    else
    {   //The robot is behind the ball
        //Go to the ball
        // normal vector from ball to goal
        //This returns a normalized vector alligned with ball and goal
        Vector2d n = utility_unitVector(goal - ball);

        // compute position 10cm behind ball, but aligned with goal.
        //subtract 10 cm (from vector) to the current ball position
        point = ball - 0.2*n;

        //This is the point we want to get to. we may be in front of the ball though
        //This would cause us to go straight for the point, hitting the ball in the wrong direction
        if(utility_vecLength(point - robot.pos) < 0.21)
        { 
              //point = avoid_robots(ball);
             // skill_goToBall(robot, point, robotId);
              skill_goToBall(robot, ball, robotId);

              return;
        }
    
    }
    //point = avoid_robots(point);
    skill_goToBall(robot, point, robotId);
    return;

    }


