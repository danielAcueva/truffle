//Header guard
#ifndef QUADRANT_H
#define QUADRANT_H

#include <stdio.h>
#include <eigen3/Eigen/Eigen>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include "globals.h"
#include "controller1.h"
#include "math.h"

using namespace std;
using namespace geometry_msgs;
using namespace Eigen;

/////////Quadrant Coordinate System\\\\\\\\\\

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

//////////////////////\\\\\\\\\\\\\\\\\\\\\\\

//get the quadrant row of an item from the XY coordinates
int get_quadrant_row(Vector2d field_item);

//get the quadrant column of an item from the XY coordinates
int get_quadrant_column(Vector2d field_item);

//Get the quadrant number from the XY coordinates.
//See the above quadrant coordinate system as a reference
int get_quadrant(Vector2d field_item);

// This function returns the center position of the quadrant
// This is useful if you need to move to a quadrant
Vector2d get_quadrant_center(int quadrant);

//end of header guard
#endif