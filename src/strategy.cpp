#include "strategy.h"

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
    Vector2d n = utility_unitVector(goal - ball);

    // compute position 10cm behind ball, but aligned with goal.
    Vector2d position = ball - 0.2*n;

    if(utility_vecLength(position - robot.pos) < 0.21)
        skill_goToPoint(robot, goal, robotId);
    else
        skill_goToPoint(robot, position, robotId);
}

