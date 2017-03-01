#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

// Line Segment Detector
#include "opencv2/core/utility.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>

#include <cmath>
#include <iostream>
#include <cv.h>
#include <highgui.h>

#include <stdlib.h> // abs
#include <algorithm>
using namespace std;
using namespace cv;

#define FIELD_WIDTH     3.53  // in meters
#define FIELD_HEIGHT    2.39 
#define ROBOT_RADIUS    0.10
#define GUI_NAME "Soccer Overhead Camera"
#define LINES_WINDOW "Trackbars"

// Mouse click parameters, empirically found
// The smaller the number, the more positive the error
// (i.e., it will be above the mouse in +y region)
#define FIELD_WIDTH_PIXELS      577.0 // measured from threshold of goal to goal
#define FIELD_HEIGHT_PIXELS     388.0 // measured from inside of wall to wall
#define CAMERA_WIDTH            865.0
#define CAMERA_HEIGHT           480.0

// These colours need to match the Gazebo materials
Scalar red[]    = {Scalar(158,  80, 200), Scalar(179,  255, 255)}; // first scalar is low, second is high
Scalar yellow[] = {Scalar(20,  128, 128), Scalar(30,  255, 255)};
Scalar green[]  = {Scalar(47,  167, 170), Scalar(95,  255, 255)};
Scalar blue[]   = {Scalar(92, 53, 214), Scalar(102, 163, 241)};
Scalar purple[] = {Scalar(145, 128, 128), Scalar(155, 255, 255)};
Scalar white[]  = {Scalar(0,  0, 156), Scalar(150,  10, 255)};
Scalar gold[]   = {Scalar(19,  106, 152), Scalar(40,  190, 215)};
Scalar diagColor[] = {Scalar(0, 0, 138), Scalar(179, 255, 255)};
Scalar pink[]    = {Scalar(172,  45, 232), Scalar(176,  255, 255)}; // ball color
Scalar field[]    = {Scalar(42,  29, 120), Scalar(80,  242, 211)}; 



Point2d center_field;

int playerColor[6];

int H_MIN = 160;
int H_MAX = 179;
int S_MIN = 80;
int S_MAX = 255;
int V_MIN = 200;
int V_MAX = 255;

Mat global_frame; 
int firstRun = 0;

Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_ADV);


/*************

    1) Could we get ride of all the subscriber publisher for everthing that is not robot and ball
            get rid of opponents and ally





**************/

// Handlers for vision position publishers
ros::Publisher home1_pub;
//ros::Publisher home2_pub;
//ros::Publisher away1_pub;
//ros::Publisher away2_pub;
ros::Publisher ball_pub;
//ros::Publisher ball_position_pub; // for publishing internally from the vision window

// Use variables to store position of objects. These variables are very
// useful when the ball cannot be seen, otherwise we'll get the position (0, 0)
geometry_msgs::Pose2D poseHome1;
//geometry_msgs::Pose2D poseHome2;
//geometry_msgs::Pose2D poseAway1;
//geometry_msgs::Pose2D poseAway2;
geometry_msgs::Pose2D poseBall;

void thresholdImage(Mat& imgHSV, Mat& imgGray, Scalar color[])
{
    inRange(imgHSV, color[0], color[1], imgGray);

    if (color == blue) // robot
    {
        imshow("Pre-Morphological", imgGray);
    }

    //erode(imgGray, imgGray, getStructuringElement(MORPH_ELLIPSE, Size(4, 4)));
    dilate(imgGray, imgGray, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
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
    Point2d center_w = (point_i - center_field);

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
    //imgHsv and imgGray are passed by reference
    thresholdImage(imgHsv, imgGray, color);
   // imshow("HSV", imgHsv);

    //imshow(GUI_NAME, imgGray);

   // waitKey(60);

    vector< vector<Point> > contours;
    vector<Moments> mm;
    vector<Vec4i> hierarchy;
    //Find countour, fill up the vector
    findContours(imgGray, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

   // imshow(GUI_NAME, imgGray);

    

    //Heirarchy shows how many objects are found.
    //We want to make sure there are two
    //we have two colored identifiers on the robot jersey

    if (hierarchy.size() != 2)
        return;

    //Add to vector of moments, the heirarchy data
    for(int i = 0; i < hierarchy.size(); i++)
        mm.push_back(moments((Mat)contours[i]));

    //Function helps with the sort
    std::sort(mm.begin(), mm.end(), compareMomentAreas);
    Moments mmLarge = mm[mm.size() - 1];
    Moments mmSmall = mm[mm.size() - 2];

    Point2d centerLarge = imageToWorldCoordinates(getCenterOfMass(mmLarge));
    Point2d centerSmall = imageToWorldCoordinates(getCenterOfMass(mmSmall));

    Point2d robotCenter = (centerLarge + centerSmall) * (1.0 / 2);
    Point2d diff = centerSmall - centerLarge;
    double angle = atan2(diff.y, diff.x);

    //convert angle to degrees
    angle = angle *180/M_PI;
    robotPose.x = robotCenter.x - 2.52 ;
    robotPose.y = robotCenter.y + 1.5 ;
    robotPose.theta = angle;

    cout << "ROBOT X_POS: " << robotPose.x << endl;
    cout << "ROBOT_Y_POS: " << robotPose.y << endl;
    cout << "THETA: " << robotPose.theta << endl;

}

// We need this function to define how to sort
// the vector. We will pass this function into the
// third parameter and it will tell it to sort descendingly.
bool wayToSort(Vec4f a, Vec4f b)
{    
    if (a[0] < b[0])
    {
        return true;
    }
    return false;
}

void getBallPose(Mat& imgHsv, Scalar color[], geometry_msgs::Pose2D& ballPose)
{
    Mat imgGray;
    thresholdImage(imgHsv, imgGray, color);

    imshow("HSV", imgHsv);

    imshow(GUI_NAME, imgGray);

    waitKey(60);


 //   imshow("HSV", imgHsv);

   // imshow(GUI_NAME, imgGray);

    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(imgGray, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    

    waitKey(60);

    if (hierarchy.size() != 1)
        return;

    Moments mm = moments((Mat)contours[0]);
    Point2d ballCenter = imageToWorldCoordinates(getCenterOfMass(mm));

    ballPose.x = ballCenter.x - 2.52;   //Center of field offset
    ballPose.y = ballCenter.y + 1.50;   //Center of field offset
    ballPose.theta = 0;

    //cout << "Ball X: " << ballPose.x << endl;
    //cout << "Ball Y: " << ballPose.y << endl;
}


void processImage(Mat frame)
{
    //Create a Mat variable imgHsv
    Mat imgHsv;
    Mat gray;
    cvtColor(frame, imgHsv, COLOR_BGR2HSV);

    /*
     * Using border values, crop the image.
     * The field is roughly 600 pixels wide and 415 pixels tall
     * The first two parameters of rect should be the top left x and y values!
    */
   // cout << "Center: " << center_field.x << ", " << center_field.y << endl;

   // int width = 640;
   // int height = 435;
    //Rect roi(center_field.x - width/2, center_field.y - height/2, width, height);
   // Mat roiImage = imgHsv(roi);

    waitKey(60);

    //Calculate the robot position. imgHsv and poseHome1 are passed by reference
    getRobotPose(imgHsv, blue, poseHome1);

    // Calculate the ball position
    getBallPose(imgHsv, pink, poseBall);



    // Publish positions
    home1_pub.publish(poseHome1);

    cout << "Ball X: " << poseBall.x << endl;
    cout << "Ball Y: " << poseBall.y << endl;
    ball_pub.publish(poseBall);

}

void getCenter(Mat frame)
{
    /*Mat imgHSV;
    Mat gray;
    Mat bgr;
    cvtColor(frame, imgHSV, COLOR_BGR2HSV);


    Mat imgGray;

    inRange(imgHSV, field[0], field[1], imgGray);

    dilate(imgGray, imgGray, getStructuringElement(MORPH_ELLIPSE, Size(20, 20)));
    //erode(imgGray, imgGray, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)));

    cvtColor(imgGray, bgr, COLOR_HSV2BGR);
    */

 

    //imshow("CENTER", imgGray);

    //waitKey(60);

   // cvtColor(bgr, gray, COLOR_BGR2GRAY);

/////////////////////////////////////////////


    Mat imgHsv;
    Mat gray;
    Mat imgGray;


    //thresholdImage(imgHsv, imgGray, color);

    //vector< vector<Point> > contours;
    //vector<Vec4i> hierarchy;
    //findContours(imgGray, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    //vector<Vec4f> lines_std;

    //Mat drawnLines(gray);
    //ls->drawSegments(drawnLines, lines_std);
    //ls->drawSegments(drawnLines, lines_std);
    //imshow("CHECK", gray);

    // Line segment detection. 
     // Detects lines for field framing
     //
    //LSD Lines detected in grayscale.
    cvtColor(frame, gray, COLOR_BGR2GRAY);
    //cvtColor(imgGray, gray, COLOR_BGR2GRAY);
    vector<Vec4f> lines_std;

    // Detect lines. Use the grayscale converted image
    ls->detect(gray, lines_std);
    Mat drawnLines(gray);
    ls->drawSegments(drawnLines, lines_std);
    imshow("CHECK", gray);



    // Sort lines into vertical and horizontal groups
    vector<Vec4f> v_lines; // (x,y) (x, y)
    vector<Vec4f> h_lines;
    vector<Vec4f> d_lines;


    float x1, x2, y1, y2;

    for (int i = 0; i < lines_std.size(); i++)
    {
        // if the x's are about the same, it's vertical
        if (abs(lines_std[i][0] - lines_std[i][2]) < (abs(lines_std[i][1] - lines_std[i][3])/30))
        {
            v_lines.push_back(lines_std[i]);
            x1 = v_lines[i][0];
            y1 = v_lines[i][1];
            x2 = v_lines[i][2];
            y2 = v_lines[i][3];
           // cout << x1 << " " << y1 << "/" << x2 << " " << y2 << endl;
        }

        // if the y's are about the same, it's horizontal
        if (abs(lines_std[i][1] - lines_std[i][3]) < (abs(lines_std[i][0] - lines_std[i][2])/50))
        {
            double length = lines_std[i][0] - lines_std[i][2];
            //cout << length << endl;
            h_lines.push_back(lines_std[i]);
        }
        if (abs((abs(lines_std[i][1] - lines_std[i][3])) - abs(lines_std[i][0] - lines_std[i][2])) < .3)
        {
            d_lines.push_back(lines_std[i]);
        }
    }

    vector<Vec4f> long_lines;
    //In this algorithm, I am assuming anything with a similar 
    //y value is on the same line
    //I believe this is a safe assumption since we have our tolerance so tight.
    //It will take a starting XY from a line, then find all the horizontal lines that line up with it
    //then finally when it finds the last line, it will use that lines ending XY to create a long line.
    //Check the end of one line with the beginning of the next
    //Store start and end points of long lines
  //  float x1, x2, y1, y2;
    int count = 0;
    bool line_to_add = false;
     for (int i = 0; i < h_lines.size(); i++)
    {
        // store the first point
        x1 = h_lines[i][0];
        y1 = h_lines[i][1];
        x2 = h_lines[i][2];
        y2 = h_lines[i][3];
      //  cout << x1 << " " << y1 << "/" << x2 << " " << y2 << endl;
        count = 0;
        line_to_add = false;
        //Start with the next line
        //Check to see if it is part of the same line
        //If so, add it to the line
        for (int j = i; j < h_lines.size() - 1; j++)
        {
            //check if the next line is part of the same horizontal line
            if (abs(h_lines[j+1][1] - h_lines[i][1]) < 10)
            {
                if ((abs(h_lines[j+1][0] - x2) < 40))// && (abs(h_lines[j+1][1] - y2) < 10))
                {
                    //It's on the same horizontal plane. Assume it's part of the line.
                    //Store the ending x and y values to add them
                    x2 = h_lines[j+1][2];
                    y2 = h_lines[j+1][3];
                    //h_lines.erase(h_lines.begin()+(j+1)); // <--  with this
                    line_to_add = true; 
                    //Already been through this line, don't need to do it again in outer loop
                    ////count++;  
                }        
            }  
            //The line is not part of the longer stitched line we are building 
            ////else
            ////{  
                ////break; 
            ////}      
        }
        //Increment the number of lines we have already used on the line stitch
        ////i += count;
        //We have an addtional point to create a line
        if (line_to_add)
        {
            long_lines.push_back(Vec4f(x1, y1, x2, y2));
        }
    }   



    vector<Vec4f> long_v_lines;
    //In this algorithm, I am assuming anything with a similar 
    //y value is on the same line
    //I believe this is a safe assumption since we have our tolerance so tight.
    //It will take a starting XY from a line, then find all the horizontal lines that line up with it
    //then finally when it finds the last line, it will use that lines ending XY to create a long line.
    //Check the end of one line with the beginning of the next
    //Store start and end points of long lines
    count = 0;
    line_to_add = false;
     for (int i = 0; i < v_lines.size(); i++)
    {
        // store the first point
        x1 = v_lines[i][0];
        y1 = v_lines[i][1];
        x2 = v_lines[i][2];
        y2 = v_lines[i][3];
       // cout << x1 << " " << y1 << "/" << x2 << " " << y2 << endl;
        count = 0;
        line_to_add = false;
        //Start with the next line
        //Check to see if it is part of the same line
        //If so, add it to the line
        for (int j = i; j < v_lines.size() - 1; j++)
        {
            //check if the next line is part of the same horizontal line
            if (abs(v_lines[j+1][0] - x1) < 10)
            {
                if ((abs(v_lines[j+1][1] - y2) < 40))// && (abs(h_lines[j+1][1] - y2) < 10))
                {
                    //It's on the same horizontal plane. Assume it's part of the line.
                    //Store the ending x and y values to add them
                    x2 = v_lines[j+1][2];
                    y2 = v_lines[j+1][3];
                    //h_lines.erase(h_lines.begin()+(j+1)); // <--  with this
                    line_to_add = true; 
                    //Already been through this line, don't need to do it again in outer loop
                    ////count++;  
                }        
            }  
            //The line is not part of the longer stitched line we are building  
        }
        //Increment the number of lines we have already used on the line stitch
        //We have an addtional point to create a line
        if (line_to_add)
        {
            long_v_lines.push_back(Vec4f(x1, y1, x2, y2));
        }
    }   

    int topCount = 0;
    float topSum = 0;
    int bottomCount = 0;
    float bottomSum = 0;


    // lines with y < 30 is top
    // lines with y > 420 is bottom
    for (int i = 0; i < long_lines.size(); i++)
    {
        if (long_lines[i][1] < 30)
        {
            topSum += long_lines[i][1];
            topCount++;
        }
        else if (long_lines[i][1] > 420)
        {
            bottomSum += long_lines[i][1];
            bottomCount++;
        }
    }
    

    float topYBorder = topSum/topCount;
    float bottomYBorder = bottomSum/bottomCount;

   // float centerY = (topYBorder + bottomYBorder)/2;
    float centerY = 245;

    float centerX = 415;

    center_field.x = centerX;
    center_field.y = centerY;
    
getCenter(frame);
  //  cout << "Center: " << center_field.x << ", " << center_field.y << endl;

    //
     // Using border values, crop the image.
     // The field is roughly 600 pixels wide and 415 pixels tall
     // The first two parameters of rect should be the top left x and y values!
     //

    /*int width = 640;
    int height = 435;
    Rect roi(centerX - width/2, centerY - height/2, width, height);
   // Mat source;
    Mat roiImage = frame(roi);


    // do a fancy visual test
    // hack for x
    long_lines.push_back(Vec4f(415 - 10, centerY - 10, 415 + 10, centerY + 10));

    Mat drawnLines(frame);
    ls->drawSegments(drawnLines, d_lines);
    //ls->drawSegments(drawnLines, lines_std);
    imshow("CHECK", roiImage);*/


   // waitKey(60);

    //Calculate the robot position. imgHsv and poseHome1 are passed by reference
    //getRobotPose(imgHsv, gold,   poseHome1);

    //publish posehome1
    //home1_pub.publish(poseHome1);
    
}

//Called when data is subscribed
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        //capture the data
        Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        //Process the image. This will process the data

        if (firstRun == 0)
        {
          //   getCenter(frame);
             firstRun = 0;
        }
        //getCenter(frame);
        processImage(frame);

        firstRun++;
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
    pink[0](0) = H_MIN;
    pink[0](1) = S_MIN;
    pink[0](2) = V_MIN;

    pink[1](0) = H_MAX;
    pink[1](1) = S_MAX;
    pink[1](2) = V_MAX;

}

void createHSVTrackbars() {
    //create window for trackbars
    namedWindow(LINES_WINDOW, 0);

    //create trackbars and insert them into window
    //3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
    //the max value the trackbar can move (eg. H_HIGH),
    //and the function that is called whenever the trackbar is moved(eg. on_trackbar)
    createTrackbar( "H_MIN", GUI_NAME, &H_MIN, 179, on_trackbar );
    createTrackbar( "H_MAX", GUI_NAME, &H_MAX, 179, on_trackbar );
    createTrackbar( "S_MIN", GUI_NAME, &S_MIN, 255, on_trackbar );
    createTrackbar( "S_MAX", GUI_NAME, &S_MAX, 255, on_trackbar );
    createTrackbar( "V_MIN", GUI_NAME, &V_MIN, 255, on_trackbar );
    createTrackbar( "V_MAX", GUI_NAME, &V_MAX, 255, on_trackbar );
}

void setPlayerColors(int val[])
{
    for(int i = 0; i < 6; i++)
        playerColor[i] = val[i];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_sim");
    ros::NodeHandle nh;

    ros::NodeHandle priv_nh("~");                       //Create private nado handle
    //priv_nh.param<string>("team", team, "home");


    // Create OpenCV Window and add a mouse callback for clicking
    namedWindow(GUI_NAME, CV_WINDOW_AUTOSIZE);
    setMouseCallback(GUI_NAME, mouseCallback, NULL);
    createHSVTrackbars();

    // Subscribe to camera
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe("/usb_cam_away/image_raw", 1, imageCallback);

    cout << "subscribed to camera" << endl;

    // Publish the robot's position (x and y)
    home1_pub = nh.advertise<geometry_msgs::Pose2D>("truffle/home1", 5);
    ball_pub = nh.advertise<geometry_msgs::Pose2D>("truffle/ball", 5);

    ros::spin();
    return 0;
}