#include "math.h"

using namespace std;
using namespace geometry_msgs; 
using namespace Eigen;

//MATH FUNCTIONS
int utility_sgn(double val)
{
    return (0 < val) - (val < 0);
}

// utility - saturate velocity
//  Saturate the commanded velocity .
Vector3d utility_saturateVelocity(Vector3d v)
{
    if(fabs(v(0)) > ROBOT_MAX_VXY)
        v(0) = utility_sgn(v(0)) * ROBOT_MAX_VXY;
    if(fabs(v(1)) > ROBOT_MAX_VXY)
        v(1) = utility_sgn(v(1)) * ROBOT_MAX_VXY;
    if(fabs(v(2)) > ROBOT_MAX_OMEGA)
        v(2) = utility_sgn(v(2)) * ROBOT_MAX_OMEGA;
    return v;
}

double utility_vecLength(Vector2d v)
{
    return sqrt(v(0)*v(0) + v(1)*v(1));
}

Vector2d utility_unitVector(Vector2d v)
{
    return v / utility_vecLength(v);
}

double utility_angleMod(double angle)
{
    while(angle < 0)
        angle += 2*M_PI;
    return fmod(angle + M_PI, (2*M_PI)) - M_PI;
}
