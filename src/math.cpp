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
    double ratio = 0.0f;
    bool x_larger = true;
    //If both velocities are above the saturation popint, then they both saturate
    //This creates a 45 degree diagonal movement. 
    //The ratio calculation below will be added to the smaller velocity (x or y)
    //It is a ratio of the larger one, so it will keep the correct angle in motion
    if (fabs(v(0)) > fabs(v(1)))
    {
        ratio = fabs(v(1)) / fabs(v(0));
        x_larger = true;
    }
    else
    {
        ratio = fabs(v(0)) / fabs(v(1));
        x_larger = false;
    }
    if(fabs(v(0)) > ROBOT_MAX_VXY)
    {
        v(0) = utility_sgn(v(0)) * ROBOT_MAX_VXY;
        //x = (condition) ? true : false;
        if (!x_larger)
        {
            v(0) = (v(0) * ratio);
        }
        //v(0) = (x_larger) ? v(0) : (v(0) * ratio);
    }
    if(fabs(v(1)) > ROBOT_MAX_VXY)
    {
        v(1) = utility_sgn(v(1)) * ROBOT_MAX_VXY;
        if (x_larger)
        {
            v(1) = (v(1) * ratio);
        }
        //v(1) = (x_larger) ? (v(1) * ratio) : v(1);
    }
    /*if(fabs(v(0)) > ROBOT_MAX_VXY)
        v(0) = utility_sgn(v(0)) * ROBOT_MAX_VXY;
    if(fabs(v(1)) > ROBOT_MAX_VXY)
        v(1) = utility_sgn(v(1)) * ROBOT_MAX_VXY;*/
    if(fabs(v(2)) > ROBOT_MAX_OMEGA)
    {
        v(2) = utility_sgn(v(2)) * ROBOT_MAX_OMEGA;
    }
    return v;
}

// utility - saturate velocity
//  Saturate the commanded velocity .
Vector3d utility_saturateVelocityHard(Vector3d v)
{
    double ratio = 0.0f;
    bool x_larger = true;
    //If both velocities are above the saturation popint, then they both saturate
    //This creates a 45 degree diagonal movement. 
    //The ratio calculation below will be added to the smaller velocity (x or y)
    //It is a ratio of the larger one, so it will keep the correct angle in motion
    if (fabs(v(0)) > fabs(v(1)))
    {
        ratio = fabs(v(1)) / fabs(v(0));
        x_larger = true;
    }
    else
    {
        ratio = fabs(v(0)) / fabs(v(1));
        x_larger = false;
    }
    if(fabs(v(0)) > 2)
    {
        v(0) = utility_sgn(v(0)) * 2;
        //x = (condition) ? true : false;
        if (!x_larger)
        {
            v(0) = (v(0) * ratio);
        }
        //v(0) = (x_larger) ? v(0) : (v(0) * ratio);
    }
    if(fabs(v(1)) > 2)
    {
        v(1) = utility_sgn(v(1)) * 2;
        if (x_larger)
        {
            v(1) = (v(1) * ratio);
        }
        //v(1) = (x_larger) ? (v(1) * ratio) : v(1);
    }
    /*if(fabs(v(0)) > ROBOT_MAX_VXY)
        v(0) = utility_sgn(v(0)) * ROBOT_MAX_VXY;
    if(fabs(v(1)) > ROBOT_MAX_VXY)
        v(1) = utility_sgn(v(1)) * ROBOT_MAX_VXY;*/
    if(fabs(v(2)) > ROBOT_MAX_OMEGA)
    {
        v(2) = utility_sgn(v(2)) * ROBOT_MAX_OMEGA;
    }
    return v;
}

//Find the vector length. Hypoteneuse
double utility_vecLength(Vector2d v)
{
    return sqrt(v(0)*v(0) + v(1)*v(1));
}

//returns normalized vector
Vector2d utility_unitVector(Vector2d v)
{
    return v / utility_vecLength(v);
}

double utility_angleMod(double angle)
{
    while(angle < 0)
    {
        angle += 360;
    }
    return fmod(angle + 180, (360) - 180);
}