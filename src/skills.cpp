#include "skills.h"

using namespace std;
using namespace geometry_msgs; 
using namespace Eigen;


// skill - go to point
//   Travels towards a point. Angle always faces the goal.
void skill_go_to_quadrant(RobotPose robot, Vector2d point, int robotId)
{
    //Find the unit vector from point to robot difference
    Vector2d pointDiff = utility_unitVector(point - robot.pos);
    Vector2d vxy;                                       //Create a vector
    if ((utility_vecLength(robot.pos - ball)) > .6){    //When we are far from ball
        vxy = pointDiff * CONTROL_K_XY;                 //Go to max velocity
    }
    else if ((utility_vecLength(robot.pos - ball)) < .1){   //When we are close to the ball
        vxy = pointDiff * CONTROL_K_XY;                     //Go to a slower velocity
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

void skill_ball_align_quadrant(RobotPose robot, int quadrant, int robotId)
{           
	Vector2d quadrant_center = get_quadrant_center(quadrant);

    // normal vector from ball to goal
    //This returns a normalized vector alligned with ball and goal
    Vector2d n = utility_unitVector(quadrant_center - ball);

    ///////STOPPED HERE

    // compute position 10cm behind ball, but aligned with goal.
    //subtract 10 cm (from vector) to the current ball position
    //int point = ball - 0.2*n;

    //This is the point we want to get to. we may be in front of the ball though
    //This would cause us to go straight for the point, hitting the ball in the wrong direction
    //if(utility_vecLength(point - robot.pos) < 0.21){ //|| 
                        //((point - robot.pos) > 0.18))
        //skill_go_to_quadrant(robot, ball, robotId);
        //return;                     //Go to the ball
    //}
    //skill_go_to_quadrant(robot, point, robotId);
    //return;
}

