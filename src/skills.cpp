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

void skill_go_to_point(RobotPose robot, Vector2d point, int robotId)
{
    Vector2d dirPoint = point - robot.pos;
    Vector2d vxy = dirPoint;// * CONTROL_K_XY;

    // control angle to face the point
    //Vector2d dirGoal = point - robot.pos;
    double theta_d = atan2(dirPoint(1), dirPoint(0));
    double omega = CONTROL_K_OMEGA * (robot.theta - theta_d); 

    // Output velocities to motors
    Vector3d v;
    v << vxy, omega;
    v = utility_saturateVelocity(v);
    publish_moveRobot(v, robotId);
}

/*void skill_go_to_quadrant(RobotPose robot, Vector2d point, int robotId)
{
    Vector2d dirPoint = point - robot.pos;
    Vector2d vxy = dirPoint;// * CONTROL_K_XY;

    // control angle to face the goal
    Vector2d dirGoal = goal - robot.pos;
    double theta_d = atan2(dirGoal(1), dirGoal(0));
    double omega = -CONTROL_K_OMEGA * (robot.theta - theta_d); 

    // Output velocities to motors
    Vector3d v;
    v << vxy, omega;
    v = utility_saturateVelocity(v);
    publish_moveRobot(v, robotId);
}*/

bool skill_ball_align_quadrant(RobotPose robot, int quadrant, int robotId)
{           
	Vector2d quadrant_center = get_quadrant_center(quadrant);

    // normal vector from ball to goal
    //This returns a normalized vector alligned with quadrant and goal
    Vector2d n = utility_unitVector(quadrant_center - ball);

    // compute position 10cm behind ball, but aligned with goal.
    //subtract 10 cm (from vector) to the current ball position
    Vector2d position = ball - 0.2*n;

    //If you are at the point, push towards the quadrant
    if(utility_vecLength(position - robot.pos) < 0.03){
        return true;
    }
    else{
        skill_go_to_point(robot, position, robotId);
    }
    return false;
}

