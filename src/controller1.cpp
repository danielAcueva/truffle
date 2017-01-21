#include <iostream>
#include <stdio.h>
#include <eigen3/Eigen/Eigen>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include "soccerref/GameState.h"
#include "globals.h"
#include "test.h"
#include "controller1.h"
#include "strategy.h"
#include "offense.h"
#include "defense.h"

using namespace std;
using namespace geometry_msgs;
using namespace Eigen;
								//2d Vector for the goal
ros::Publisher motor_pub1;					//Publish motor_pub1
ros::Publisher motor_pub2;					//Publish motor_pub2

ros::Subscriber vsub_ally1, vsub_ally2;		//Subscribe to vsub_ally1, vsub_ally2
ros::Subscriber vsub_opp1, vsub_opp2;		//Subscribe to vsub_opp1, vsub_opp2
ros::Subscriber vsub_ball;					//Subscribe to the ball
ros::Subscriber game_state_sub;				//subscribe to the game state
soccerref::GameState gameState;				//Soccer referee

string team;
Vector2d goal;
RobotPose ally1, ally2;						//Ally's are made of struct pose
RobotPose opp1, opp2;						//Opponents are made of struct pose
Vector2d ball;								//Ball is a 2d vector
Vector2d ally1_startingPos;					//starting position. 2d vector
Vector2d ally2_startingPos;					//starting position. 2d vector

//function to move the robot 
void publish_moveRobot(Vector3d v_world, int robotId)
{
	//v_world is a 3d vector
    geometry_msgs::Twist v;					//Create a geometry msg. twist. v
    v.linear.x = v_world(0);				//Move X
    v.linear.y = v_world(1);				//Move Y
    v.angular.z = v_world(2);				//Move z

    if(robotId == 1)						//If robot number 1
        motor_pub1.publish(v);				//Publish the move
    else if(robotId == 2)					//If robot number 2
        motor_pub2.publish(v);				//Publish the move
}

void param_init()
{
    goal << FIELD_WIDTH/2, 0;				//Set the goal
}

//MATH FUNCTIONS
/*int utility_sgn(double val)
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
}*/

//VISION FUNCTIONS
RobotPose utility_toRobotPose(Pose2D robot)
{
    // Flip coordinates if team is away or if we've swapped sides
    if((team == "away") ^ gameState.second_half)
    {
        robot.x = -robot.x;
        robot.y = -robot.y;
        robot.theta = utility_angleMod(robot.theta + M_PI);
    }
    Vector2d pos;
    pos << robot.x, robot.y;
    return (RobotPose){pos, robot.theta};
}

Vector2d utility_toBallPose(Pose2D ball)
{
    // Flip coordinates if team is away or if we've swapped sides
    if((team == "away") ^ gameState.second_half)
    {
        ball.x = -ball.x;
        ball.y = -ball.y;
    }
    Vector2d pos;
    pos << ball.x, ball.y;
    return pos;
}

// skill - follow ball on line
//   Follows the y-position of the ball, while maintaining x-position at x_pos. 
//   Angle always faces the goal.
/*void skill_followBallOnLine(RobotPose robot, Vector2d ball, double x_pos, int robotId)
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
}*/

void visionCallback(const geometry_msgs::Pose2D::ConstPtr &msg, const std::string& robot)
{
    if(robot == "ally1")
        ally1 = utility_toRobotPose(*msg);

    else if(robot == "ally2")
        ally2 = utility_toRobotPose(*msg);

    else if(robot == "opponent1")
        opp1 = utility_toRobotPose(*msg);

    else if(robot == "opponent2")
        opp2 = utility_toRobotPose(*msg);

    else if(robot == "ball")
        ball = utility_toBallPose(*msg);
}

void gameStateCallback(const soccerref::GameState::ConstPtr &msg)
{
    gameState = *msg;
}

int main(int argc, char **argv)
{
    param_init();										//Init parameters
    ros::init(argc, argv, "home");						//Init ros. Call it home
    ros::NodeHandle nh;									//Create node handle

    // Private node handle to get whether we are home or away.
    // Having the nh private properly namespaces it.
    ros::NodeHandle priv_nh("~");						//Create private nado handle
    priv_nh.param<string>("team", team, "home");

    vsub_ally1 = nh.subscribe<geometry_msgs::Pose2D>("ally1_vision", 1, boost::bind(visionCallback, _1, "ally1"));
    vsub_ally2 = nh.subscribe<geometry_msgs::Pose2D>("ally2_vision", 1, boost::bind(visionCallback, _1, "ally2"));
    vsub_opp1 = nh.subscribe<geometry_msgs::Pose2D>("opponent1_vision", 1, boost::bind(visionCallback, _1, "opponent1"));
    vsub_opp2 = nh.subscribe<geometry_msgs::Pose2D>("opponent2_vision", 1, boost::bind(visionCallback, _1, "opponent2"));
    vsub_ball = nh.subscribe<geometry_msgs::Pose2D>("ball_vision", 1, boost::bind(visionCallback, _1, "ball"));
    vsub_ball = nh.subscribe<geometry_msgs::Pose2D>("ball_vision", 1, boost::bind(visionCallback, _1, "ball"));
    game_state_sub = nh.subscribe<soccerref::GameState>("/game_state", 1, gameStateCallback);
    motor_pub1 = nh.advertise<geometry_msgs::Twist>("ally1/vel_cmds", 5);
    motor_pub2 = nh.advertise<geometry_msgs::Twist>("ally2/vel_cmds", 5);

    // This is sort of ad-hoc (would be much better to be a parameter) but it works for now
    ally1_startingPos << -0.5, 0;
    ally2_startingPos << -1.0, 0;

	int count = 0;
	
    ros::Rate loop_rate(30);							//30 cycles a second
    while(ros::ok())									//Run until ctrl+c
    {
        if (gameState.play)								//We are playing. Play ball!
        {
	
			/*//Test values
			Vector3d zeroVel;							
            zeroVel << 0, .05, 5;							//Mo more movements
            publish_moveRobot(zeroVel, 1);						//keep robot still. move with no parameters
            publish_moveRobot(zeroVel, 2);						//keep robot still. Move with no parameters
			*/
			
			///////////////////////////////////////////////////////////////////////
			///////////////////FOLLOW THE BALL/////////////////////////////////////
			//Robot 1
			//test_followBall(ally1, ball, 1);
			//Robot 2
			//test_followBall(ally2, ball, 2);

			
            /*********************************************************************/
            
            // Choose strategies

            // robot #1 positions itself behind ball and rushes the goal.
            playOffense(1);
            //play_rushGoal(ally1, ball, 1);

            // robot #2 stays on line, following the ball, facing the goal
            playDefense(2);
            //skill_followBallOnLine(ally2, ball, -2 * FIELD_WIDTH / 6, 2);
			
            /*********************************************************************/
        }
        else if (gameState.reset_field)					//Reset the game
        {
            skill_goToPoint(ally1, ally1_startingPos, 1);	
            skill_goToPoint(ally2, ally2_startingPos, 2);
        }
        else 											//paused - stop moving
        {
            Vector3d zeroVel;							
            zeroVel << 0, 0, 0;							//Mo more movements
            publish_moveRobot(zeroVel, 1);						//keep robot still. move with no parameters
            publish_moveRobot(zeroVel, 2);						//keep robot still. Move with no parameters
        }
        // process any callbacks
        ros::spinOnce();

        // force looping at a constant rate
        loop_rate.sleep();
    }

    // Clean up
    Vector3d zeroVel;
    zeroVel << 0, 0, 0;
    publish_moveRobot(zeroVel, 1);
    publish_moveRobot(zeroVel, 2);
    return 0;
}
