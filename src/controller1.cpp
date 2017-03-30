#include <iostream>
#include <stdio.h>
#include <eigen3/Eigen/Eigen>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
//#include <soccerref/GameState.h>
#include "globals.h"
#include "helper.h"
#include "controller1.h"
#include "strategy/strategy.h"
#include "offense.h"
#include "defense.h"
#include "offensive_plays/O_Dribble_Ball.h"
#include "offensive_plays/O_Intercept_Avoid.h"

using namespace std;
using namespace geometry_msgs;
using namespace Eigen;
								//2d Vector for the goal
ros::Publisher motor_pub1;					//Publish motor_pub1
ros::Publisher motor_pub2;					//Publish motor_pub2

ros::Subscriber vsub_ally1, vsub_ally2;		//Subscribe to vsub_ally1, vsub_ally2
ros::Subscriber vsub_opp1, vsub_opp2;		//Subscribe to vsub_opp1, vsub_opp2
ros::Subscriber ball_pub;					//Subscribe to the ball
//ros::Subscriber game_state_sub;				//subscribe to the game state
//soccerref::GameState gameState;				//Soccer referee

string team;
Vector2d goal;
RobotPose ally1, ally2;						//Ally's are made of struct pose
RobotPose opp1, opp2;						//Opponents are made of struct pose
Vector2d ball;								//Ball is a 2d vector
Vector2d ball_expected;                              //Ball is a 2d vector
Vector2d ally1_startingPos;					//starting position. 2d vector
Vector2d ally2_startingPos;					//starting position. 2d vector

//function to move the robot 
void publish_moveRobot(Vector3d v_world, int robotId)
{
	//v_world is a 3d vector
    geometry_msgs::Twist v;					//Create a geometry msg. twist. v
    /*if((team == "away") ^ gameState.second_half)
    {
        v.linear.x = -v_world(0);               //Move X
    }
    else
    {
        v.linear.x = v_world(0);               //Move X
    }*/
    //////HOME
    v.linear.x = v_world(0);               //Move X
    //////AWAY 
    //v.linear.x = -v_world(0);				//Move X
    ////
    v.linear.y = v_world(1);				//Move Y
    v.angular.z = v_world(2);				//Move z

    if(robotId == 1)						//If robot number 1
    {
        motor_pub1.publish(v);				//Publish the move
        //cout << "Motor Publish: X " << v.linear.x << " Y " << v.linear.y << " Z " << v.angular.z << endl;
    }
    else if(robotId == 2)					//If robot number 2
    {
        motor_pub2.publish(v);				//Publish the move
        //cout << "Motor Publish 2: X " << v.linear.x << " Y " << v.linear.y << " Z " << v.angular.z << endl;
    }
}

void param_init()
{
    goal << FIELD_WIDTH/2, 0;				//Set the goal
}

//VISION FUNCTIONS
// also look up "TMUX"
RobotPose utility_toRobotPose(Pose2D robot)
{
    /*
    // Flip coordinates if team is away or if we've swapped sides
    if((team == "away") ^ gameState.second_half)
    {
        robot.x = -robot.x;
        robot.theta  = utility_angleMod(robot.theta + 180);
    }
    */
    ///////HOME COMMENT OUT AWAY
    ///////AWAY
    //robot.x = -robot.x;
    //robot.theta  = utility_angleMod(robot.theta + 180);
    /////////////
    Vector2d pos;
    pos << robot.x, robot.y;
    return (RobotPose){pos, robot.theta};

}

Vector2d utility_toBallPose(Pose2D ball)
{
    // Flip coordinates if team is away or if we've swapped sides
  /*  if((team == "away") ^ gameState.second_half)
    {
        ball.x = -ball.x;
    }
    */
    ///////HOME COMMENT OUT AWAY
    ///////AWAY
    //ball.x = -ball.x;
    /////////////////

    Vector2d pos;
    pos << ball.x, ball.y;
    return pos;
}

void visionCallback(const geometry_msgs::Pose2D::ConstPtr &msg, const std::string& robot)
{
    if(robot == "ally1")
    {
        ally1 = utility_toRobotPose(*msg);
    }
    else if(robot == "ally2")
    {
        ally2 = utility_toRobotPose(*msg);
        intercept_avoid_tick();                         //Tick function for intercept with avoidance
    }
    else if(robot == "opponent1")
    {
        opp1 = utility_toRobotPose(*msg);
    }
    else if(robot == "opponent2")
    {
        opp2 = utility_toRobotPose(*msg);
    }
    else if(robot == "ball")
    {
        ball = utility_toBallPose(*msg);
        push_ball_values(ball);
        ball(0) = ball(0) + (avg_distance_between_samples()(0)*5);
        ball(1) = ball(1) + (avg_distance_between_samples()(1)*5);
    }
  //  cout << "robot: x " << ally1.pos(0) << " y " << ally1.pos(1) << endl;
  //  cout << "ball: x " << ball(0) << " y " << ball(1) << endl;
}

//void gameStateCallback(const soccerref::GameState::ConstPtr &msg)
//{
  //  gameState = *msg;
//}

int main(int argc, char **argv)
{
    cout << "frickin go" << endl;
    param_init();										//Init parameters
    //ros::init(argc, argv, "home");						//Init ros. Call it home
    ros::init(argc, argv, "home");                      //Init ros. Call it home
    ros::NodeHandle nh;									//Create node handle

    // Private node handle to get whether we are home or away.
    // Having the nh private properly namespaces it.
    ros::NodeHandle priv_nh("~");						//Create private nado handle
    //priv_nh.param<string>("team", team, "home");
    priv_nh.param<string>("team", team, "home");

    vsub_ally1 = nh.subscribe<geometry_msgs::Pose2D>("ally1_vision", 1, boost::bind(visionCallback, _1, "ally1"));
    vsub_ally2 = nh.subscribe<geometry_msgs::Pose2D>("ally2_vision", 1, boost::bind(visionCallback, _1, "ally2"));
    vsub_opp1 = nh.subscribe<geometry_msgs::Pose2D>("opponent1_vision", 1, boost::bind(visionCallback, _1, "opponent1"));
    vsub_opp2 = nh.subscribe<geometry_msgs::Pose2D>("opponent2_vision", 1, boost::bind(visionCallback, _1, "opponent2"));
    ball_pub = nh.subscribe<geometry_msgs::Pose2D>("ball_vision", 1, boost::bind(visionCallback, _1, "ball"));
    //game_state_sub = nh.subscribe<soccerref::GameState>("/game_state", 1, gameStateCallback);
    motor_pub1 = nh.advertise<geometry_msgs::Twist>("ally1/vel_cmds", 5);
    ////motor_pub1 = nh.advertise<geometry_msgs::Twist>("truffle/vel_cmds", 5);

    motor_pub2 = nh.advertise<geometry_msgs::Twist>("ally2/vel_cmds", 5);

    // This is sort of ad-hoc (would be much better to be a parameter) but it works for now
    ally1_startingPos << -0.5, 0;
    ally2_startingPos << -1.0, 0;

	int count = 0;

    reset_ball_values();
	
    ros::Rate loop_rate(30);							//30 cycles a second
    while(ros::ok())									//Run until ctrl+c
    {
       // if (ally1.pos(0) != 0)
            //cout << "robot: x " << ally1.pos(0) << " y " << ally1.pos(1) << endl;
       // if (ball(0) != 0)
            //cout << "ball: x " << ball(0) << " y " << ball(1) << endl;
        //cout << "x " << ally1.pos(0) << " y " << ally1.pos(1) << endl;
        //intercept_avoid_tick();                         //Tick function for intercept with avoidance
        /*if (gameState.play)								//We are playing. Play ball!
        {
            push_ball_values(ball);
            Vector2d avgValues = avg_distance_between_samples(); 
            intercept_avoid_tick();                         //Tick function for intercept with avoidance
            dribble_ball_tick();                            //Tick function for dribble
	
            // Choose strategies

            // robot #1 positions itself behind ball and rushes the goal.
            playOffense(1);
            //play_rushGoal(ally1, ball, 1);

            Vector2d predict;
            predict = ball - (avgValues*100);
            predict(1) = predict(1) - .5;
            //skill_go_to_point(ally2, predict, 2);
            //playDefense(2);
            cout << "PLAY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
			
           
        }
        else if (gameState.reset_field)					//Reset the game
        {
            skill_goToPoint(ally1, ally1_startingPos, 1);	
            skill_goToPoint(ally2, ally2_startingPos, 2);
            cout << "RESET FIELD!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
        }
        else 											//paused - stop moving
        {
            Vector3d zeroVel;							
            zeroVel << 0, 0, 0;							//Mo more movements
            publish_moveRobot(zeroVel, 1);						//keep robot still. move with no parameters
            publish_moveRobot(zeroVel, 2);						//keep robot still. Move with no parameters
        }
        // process any callbacks
*/
       // playDefense(1);

        //skill_followBallOnLine1(ally1, ball, -2 * FIELD_WIDTH / 6, 1);
        //playDefense(1);
       // playDefense(2);
        playOffense(2);
        //playDefense(2);
        /*Vector3d v;
        v << 1, 0, -1;
        v = utility_saturateVelocity(v);
        publish_moveRobot(v, 1);*/

       // Vector3d v;
        //v << 0.5, 0, 0;
        //publish_moveRobot(v, 2);


        ros::spinOnce();

        // force looping at a constant rate
        loop_rate.sleep();
    }

    // Clean up
    //Vector3d zeroVel;
    //zeroVel << 0, 0, 0;
    //publish_moveRobot(zeroVel, 1);
    //publish_moveRobot(zeroVel, 2);
    return 0;
}
