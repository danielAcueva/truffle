#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <cmath>
#include <iostream>
#include <cv.h>
#include <highgui.h>

using namespace std;
using namespace cv;

#define FIELD_WIDTH     3.53  // in meters
#define FIELD_HEIGHT    2.39 
#define ROBOT_RADIUS    0.10
#define GUI_NAME "Soccer Overhead Camera"
#define TRACK_NAME "Trackbars"

// Mouse click parameters, empirically found
// The smaller the number, the more positive the error
// (i.e., it will be above the mouse in +y region)
#define FIELD_WIDTH_PIXELS      577.0 // measured from threshold of goal to goal
#define FIELD_HEIGHT_PIXELS     388.0 // measured from inside of wall to wall
#define CAMERA_WIDTH            640.0
#define CAMERA_HEIGHT           480.0

// These colours need to match the Gazebo materials
Scalar red[]    = {Scalar(158,  80, 200), Scalar(179,  255, 255)}; // first scalar is low, second is high
Scalar yellow[] = {Scalar(20,  128, 128), Scalar(30,  255, 255)};
Scalar green[]  = {Scalar(47,  167, 170), Scalar(95,  255, 255)};
Scalar blue[]   = {Scalar(115, 128, 128), Scalar(125, 255, 255)};
Scalar purple[] = {Scalar(145, 128, 128), Scalar(155, 255, 255)};

// Scalar red[]    = {Scalar(158,  93, 181), Scalar(179,  201, 253)}; // first scalar is low, second is high
// calculated above

int playerColor[6];

int H_MIN = 160;
int H_MAX = 179;
int S_MIN = 80;
int S_MAX = 255;
int V_MIN = 200;
int V_MAX = 255;

Mat global_frame; 
//Scalar red[]    = {Scalar(160,  80, 200), Scalar(179,  255, 255)}; // first scalar is low, second is high

// Handlers for vision position publishers
ros::Publisher home1_pub;
ros::Publisher home2_pub;
ros::Publisher away1_pub;
ros::Publisher away2_pub;
ros::Publisher ball_pub;
ros::Publisher ball_position_pub; // for publishing internally from the vision window

// Use variables to store position of objects. These variables are very
// useful when the ball cannot be seen, otherwise we'll get the position (0, 0)
geometry_msgs::Pose2D poseHome1;
geometry_msgs::Pose2D poseHome2;
geometry_msgs::Pose2D poseAway1;
geometry_msgs::Pose2D poseAway2;
geometry_msgs::Pose2D poseBall;

void thresholdImage(Mat& imgHSV, Mat& imgGray, Scalar color[])
{
    inRange(imgHSV, color[0], color[1], imgGray);

    erode(imgGray, imgGray, getStructuringElement(MORPH_ELLIPSE, Size(2, 2)));
    dilate(imgGray, imgGray, getStructuringElement(MORPH_ELLIPSE, Size(2, 2)));
}

Point2d getCenterOfMass(Moments moment)
{
    double m10 = moment.m10;
    double m01 = moment.m01;
    double mass = moment.m00;
    double x = m10 / mass;
    double y = m01 / mass;
    return Point2d(x, y);
}

bool compareMomentAreas(Moments moment1, Moments moment2)
{
    double area1 = moment1.m00;
    double area2 = moment2.m00;
    return area1 < area2;
}

Point2d imageToWorldCoordinates(Point2d point_i)
{
    Point2d centerOfField(CAMERA_WIDTH/2, CAMERA_HEIGHT/2);
    Point2d center_w = (point_i - centerOfField);

    // You have to split up the pixel to meter conversion
    // because it is a rect, not a square!
    center_w.x *= (FIELD_WIDTH/FIELD_WIDTH_PIXELS);
    center_w.y *= (FIELD_HEIGHT/FIELD_HEIGHT_PIXELS);

    // Reflect y
    center_w.y = -center_w.y;
    
    return center_w;
}

void getRobotPose(Mat& imgHsv, Scalar color[], geometry_msgs::Pose2D& robotPose)
{
    Mat imgGray;
    thresholdImage(imgHsv, imgGray, color);

    vector< vector<Point> > contours;
    vector<Moments> mm;
    vector<Vec4i> hierarchy;
    findContours(imgGray, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

        imshow(TRACK_NAME, imgGray);

        waitKey(60);

    if (hierarchy.size() != 2)
        return;

    for(int i = 0; i < hierarchy.size(); i++)
        mm.push_back(moments((Mat)contours[i]));

    std::sort(mm.begin(), mm.end(), compareMomentAreas);
    Moments mmLarge = mm[mm.size() - 1];
    Moments mmSmall = mm[mm.size() - 2];

    Point2d centerLarge = imageToWorldCoordinates(getCenterOfMass(mmLarge));
    Point2d centerSmall = imageToWorldCoordinates(getCenterOfMass(mmSmall));

  //  cout << "Large: " << centerLarge.x << ", " << centerLarge.y << endl;
  //  cout << "Small: " << centerSmall.x << ", " << centerSmall.y << endl;


    Point2d robotCenter = (centerLarge + centerSmall) * (1.0 / 2);
    Point2d diff = centerSmall - centerLarge;
    double angle = atan2(diff.y, diff.x);

    //convert angle to degrees
    angle = angle *180/M_PI;
    robotPose.x = robotCenter.x;
    robotPose.y = robotCenter.y;
    robotPose.theta = angle;


    cout << "ROBOT X_POS: " << robotPose.x << endl;
    //cout << "ROBOT_Y_POS: " << robotPose.y << endl;
  //  cout << "THETA: " << robotPose.theta << endl;

}

void processImage(Mat frame)
{
    Mat imgHsv;
    cvtColor(frame, imgHsv, COLOR_BGR2HSV);

    getRobotPose(imgHsv, red,   poseHome1);

    home1_pub.publish(poseHome1);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        processImage(frame);
        imshow(GUI_NAME, frame);
       // imshow(TRACK_NAME, frame);

        waitKey(60);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void mouseCallback(int event, int x, int y, int flags, void* userdata) {
    static bool mouse_left_down = false;

    if (event == EVENT_LBUTTONDOWN) {
        mouse_left_down = true;
        Point2d point_meters = imageToWorldCoordinates(Point2d(x, y));
        char buffer[50];
        sprintf(buffer, "Location: (%.3f m, %.3f m)", point_meters.x, point_meters.y);
        displayStatusBar(GUI_NAME, buffer, 10000);

    } else if (event == EVENT_MOUSEMOVE) {
     //   if (mouse_left_down) sendBallMessage(x, y);

    } else if (event == EVENT_LBUTTONUP) {
      //  sendBallMessage(x, y);
        mouse_left_down = false;
    }
    
}

// This function is called whenever a trackbar changes
void on_trackbar( int, void* ) {
 // Scalar red[]    = {Scalar(160,  80, 200), Scalar(179,  255, 255)}; // first scalar is low, second is high

red[0](0) = H_MIN;
red[0](1) = S_MIN;
red[0](2) = V_MIN;

red[1](0) = H_MAX;
red[1](1) = S_MAX;
red[1](2) = V_MAX;


}

void createHSVTrackbars() {
    //create window for trackbars
    namedWindow(TRACK_NAME,0);

    //create trackbars and insert them into window
    //3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
    //the max value the trackbar can move (eg. H_HIGH),
    //and the function that is called whenever the trackbar is moved(eg. on_trackbar)
    createTrackbar( "H_MIN", TRACK_NAME, &H_MIN, 179, on_trackbar );
    createTrackbar( "H_MAX", TRACK_NAME, &H_MAX, 179, on_trackbar );
    createTrackbar( "S_MIN", TRACK_NAME, &S_MIN, 255, on_trackbar );
    createTrackbar( "S_MAX", TRACK_NAME, &S_MAX, 255, on_trackbar );
    createTrackbar( "V_MIN", TRACK_NAME, &V_MIN, 255, on_trackbar );
    createTrackbar( "V_MAX", TRACK_NAME, &V_MAX, 255, on_trackbar );
}

void setPlayerColors(int val[])
{
    for(int i = 0; i < 6; i++)
        playerColor[i] = val[i];
}

int main(int argc, char **argv)
{

    //setPlayerColors(GREEN_C);

    ros::init(argc, argv, "vision_sim");
    ros::NodeHandle nh;

    // Create OpenCV Window and add a mouse callback for clicking
    namedWindow(GUI_NAME, CV_WINDOW_AUTOSIZE);
    setMouseCallback(GUI_NAME, mouseCallback, NULL);
   createHSVTrackbars();
       //    imshow(TRACK_NAME, global_frame);
       // waitKey(60);

    cout << "ok" << endl;

    // Subscribe to camera
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe("/usb_cam_away/image_raw", 1, imageCallback);

    cout << "subscribed to camera" << endl;

    // Publish the robot's position (x and y)
    home1_pub = nh.advertise<geometry_msgs::Pose2D>("/vision/home1", 5);


    ros::spin();
    return 0;
}