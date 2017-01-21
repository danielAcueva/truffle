#include "test.h"

using namespace std;
using namespace geometry_msgs; 
using namespace Eigen;

//////////////////////////////////////////////////////////////////////////

//TEST. Follow the ball!
void test_followBall(RobotPose robot, Vector2d ball, int robotId)
{
	//Find x velocity relative to the ball
    //CONTROL_K_XY is constant value of 5. multiplied by offset.
    double vx = CONTROL_K_XY * (ball(0) - robot.pos(0));

    //Find y velocity relative to the ball
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
