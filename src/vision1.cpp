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

#include <cv.h>


using namespace std;
using namespace cv;
using namespace geometry_msgs;

#define FIELD_WIDTH     3.175  // in meters
#define FIELD_HEIGHT    2.22 
#define ROBOT_RADIUS    0.10
#define GUI_NAME "Soccer Overhead Camera"
#define LINES_WINDOW "Trackbars"

// Mouse click parameters, empirically found
// The smaller the number, the more positive the error
// (i.e., it will be above the mouse in +y region)
#define FIELD_WIDTH_PIXELS      642.0 // measured from threshold of goal to goal
#define FIELD_HEIGHT_PIXELS     440.0 // measured from inside of wall to wall


// Jersey colors
//Scalar green[]  = {Scalar(31, 24, 229), Scalar(38, 44, 255)};
Scalar green[]  = {Scalar(30, 19, 220), Scalar(44, 47, 242)}; // green sucks

Scalar blue[]   = {Scalar(94, 80, 222), Scalar(114, 107, 255)}; // 10:00 pm. Pretty dark
Scalar red[]    = {Scalar(5,  101, 220), Scalar(15,  119, 227)}; // first scalar is low, second is high
Scalar purple[] = {Scalar(117, 22, 213), Scalar(135, 68, 255)};

// Ball color
Scalar pink[]    = {Scalar(170,  24, 213), Scalar(179,  72, 255)};

Scalar field[]    = {Scalar(42,  29, 120), Scalar(80,  242, 211)}; 

float topYBorder;
float bottomYBorder;

float leftXBorder;
float rightXBorder;

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


int thresh_val_home1 = 0;
int thresh_val_home2 = 0;
int thresh_val_away1 = 0;
int thresh_val_away2 = 0;


//Ptr<LineSegmentDetector> ls2 = createLineSegmentDetector(LSD_REFINE_ADV, 0.3, 0.6, 2.0, 13.0, 20); // better?
Ptr<LineSegmentDetector> ls2 = createLineSegmentDetector(LSD_REFINE_ADV, 0.3, 1, 2.0, 15.0, 20, 0.1, 2048); // better?


/*

C++: Ptr<LineSegmentDetector> createLineSegmentDetector
(int refine, double scale, double sigma_scale, 
 double quant, double ang_th, double log_eps, 
 double density_th, int n_bins
)

LSD_REFINE_NONE - No refinement applied.
LSD_REFINE_STD - Standard refinement is applied. E.g. breaking arches into smaller straighter line approximations.
LSD_REFINE_ADV - Advanced refinement. Number of false alarms is calculated, lines are refined through increase of precision, decrement in size, etc.
scale – The scale of the image that will be used to find the lines. Range (0..1].

sigma_scale – Sigma for Gaussian filter. It is computed as sigma = _sigma_scale/_scale.
quant – Bound to the quantization error on the gradient norm.
ang_th – Gradient angle tolerance in degrees.
log_eps – Detection threshold: -log10(NFA) > log_eps. Used only when advancent refinement is chosen.
density_th – Minimal density of aligned region points in the enclosing rectangle. (0 to 1)
n_bins – Number of bins in pseudo-ordering of gradient modulus.
*/
/*************

    1) Could we get ride of all the subscriber publisher for everthing that is not robot and ball
            get rid of opponents and ally
            NO




**************/

// Handlers for vision position publishers
ros::Publisher home1_pub;
ros::Publisher home2_pub;
ros::Publisher away1_pub;
ros::Publisher away2_pub;
ros::Publisher ball_pub;
//ros::Publisher ball_position_pub; // for publishing internally from the vision window

// Use variables to store position of objects. These variables are very
// useful when the ball cannot be seen, otherwise we'll get the position (0, 0)
Pose2D poseHome1;
Pose2D poseHome2;
Pose2D poseAway1;
Pose2D poseAway2;
Pose2D poseBall;

void thresholdImage(Mat& imgHSV, Mat& imgGray, Scalar color[])
{

    inRange(imgHSV, color[0], color[1], imgGray);

    if (color == blue) // robot
    {
        //imshow("Pre-Morphological", imgGray);
    }

    //erode(imgGray, imgGray, getStructuringElement(MORPH_ELLIPSE, Size(4, 4)));


    if (color == pink)
    {
        dilate(imgGray, imgGray, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));

    }
    else
    {
        dilate(imgGray, imgGray, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    }
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
    center_w.x *= (FIELD_WIDTH/FIELD_WIDTH_PIXELS); //-.005; // quick hack
    center_w.y *= (FIELD_HEIGHT/FIELD_HEIGHT_PIXELS);


    // Reflect y
    center_w.y = -center_w.y;
    
    return center_w;
}

Point2d centerToWorldCoordinates(Point2d point_i)
{
    Point2d center_w = (point_i - center_field);

    // You have to split up the pixel to meter conversion
    // because it is a rect, not a square!
    center_w.x *= (FIELD_WIDTH/FIELD_WIDTH_PIXELS);
    center_w.y *= (FIELD_HEIGHT/FIELD_HEIGHT_PIXELS);

    //cout << "x: " << center_w.x << ", Y: " << center_w.y << endl;

    // Reflect y
    center_w.y = -center_w.y;
    
    return center_w;
}

void getRobotPose(Mat& imgHsv, Scalar color[], Pose2D &robotPose)
{

    Mat imgGray;

    //imgHsv and imgGray are passed by reference
    thresholdImage(imgHsv, imgGray, color);
    //imshow("hsv", imgHsv);


   // imshow("threshold me", imgGray);
   // waitKey(60);

    // grayscale source image
    // threshold value which is used to classify the pixel values. 
    // maxVal which represents the value to be given if pixel value is more than (sometimes less than) the threshold value.
    // Adaptive Method - It decides how thresholding value is calculated.
    // Block Size - It decides the size of neighbourhood area.
    // C - It is just a constant which is subtracted from the mean or weighted mean calculated.

    //adaptiveThreshold(InputArray src, OutputArray dst, double maxValue, int adaptiveMethod, int thresholdType, int blockSize, double C)
    //void cvAdaptiveThreshold(const CvArr* src, CvArr* dst, double max_value, int adaptive_method=CV_ADAPTIVE_THRESH_MEAN_C, int threshold_type=CV_THRESH_BINARY, int block_size=3, double param1=5 )
    //Mat th3 = adaptiveThreshold(imgGray, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 11, 2)
    
    //Mat blur;

    //GaussianBlur(imgGray, blur, (5,5),0)
    Mat th3;

    // This call returns a threshold value
    // Only use OTSU for the FIRST TIME
    // make sure the robot is on the field
    // if it isn't, OTSU will choose a bogus value
    // afterwards, use the returned threshold value from now on
    // Also, Hamad says they did something where in the beginning, they click their robot (with imshow)
    // which gets the HSV values, then they do like +- on the values do get a

    if (&robotPose == &poseHome1)
    {


        if (!firstRun)
        {
           thresh_val_home1 = threshold(imgGray, th3, 0, 255, THRESH_BINARY | THRESH_OTSU);
        }
        else
        {
           threshold(imgGray, th3, thresh_val_home1, 255, THRESH_BINARY);
        }
          //  imshow("adapt me", th3);
          //  waitKey(60); 

    }
    else if (&robotPose == &poseHome2)
    {
        if (!firstRun)
        {
           thresh_val_home2 = threshold(imgGray, th3, 0, 255, THRESH_BINARY | THRESH_OTSU);
        }
        else
        {
           threshold(imgGray, th3, thresh_val_home2, 255, THRESH_BINARY);
        }
    }
    else if (&robotPose == &poseAway1)
    {
        if (!firstRun)
        {
           thresh_val_away1 = threshold(imgGray, th3, 0, 255, THRESH_BINARY | THRESH_OTSU);
        }
        else
        {
           threshold(imgGray, th3, thresh_val_away1, 255, THRESH_BINARY);
        }
 
    }
    else if (&robotPose == &poseAway2)
    {
        if (!firstRun)
        {
           thresh_val_away2 = threshold(imgGray, th3, 0, 255, THRESH_BINARY | THRESH_OTSU);
        }
        else
        {
           threshold(imgGray, th3, thresh_val_away2, 255, THRESH_BINARY);
        }

    }   

    vector< vector<Point> > contours;
    vector<Moments> mm;
    vector<Vec4i> hierarchy;

    //Find countour, fill up the vector
    findContours(th3, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    //Heirarchy shows how many objects are found.
    //We want to make sure there are two
    //we have two colored identifiers on the robot jersey
    if (hierarchy.size() != 2)
    {
        //cout << "bad hierarchy" << endl;
        return;
    }

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
    robotPose.x = robotCenter.x;
    robotPose.y = robotCenter.y;
    robotPose.theta = angle;
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


    imshow("adapt me", imgGray);
    waitKey(60); 


    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(imgGray, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
   
    if (hierarchy.size() != 1)
    {
        //cout << "hierarchy fail" << endl;
        return;
    }
    else
    {
       // cout << "hierarchy good" << endl;
    }



    Moments mm = moments((Mat)contours[0]);
    Point2d ballCenter = imageToWorldCoordinates(getCenterOfMass(mm));

    ballPose.x = ballCenter.x;   //Center of field offset
    ballPose.y = ballCenter.y;   //Center of field offset
    ballPose.theta = 0;

   // cout << "[Vision] Ball: " << ballPose.x << ", " << ballPose.y << endl;
}


void processImage(Mat frame)
{
    //Create a Mat variable imgHsv
    Mat imgHsv;
    Mat gray;
    cvtColor(frame, imgHsv, COLOR_BGR2HSV);


    //Calculate the robot position. imgHsv and poseHome1 are passed by reference
    getRobotPose(imgHsv, red, poseHome1);
    getRobotPose(imgHsv, purple, poseHome2);
    getRobotPose(imgHsv, green, poseAway1);
    getRobotPose(imgHsv, blue, poseAway2);

    //Print out robot positions
   // cout << "[Vision] Robot Home 1: " << poseHome1.x << ", " << poseHome1.y << ", " << poseHome1.theta << endl;
    cout << "[Vision] Robot Home 2: " << poseHome2.x << ", " << poseHome2.y << ", " << poseHome2.theta << endl;
   // cout << "[Vision] Robot Away 1: " << poseAway1.x << ", " << poseAway1.y << ", " << poseAway1.theta << endl;
  //  cout << "[Vision] Robot Away 2: " << poseAway2.x << ", " << poseAway2.y << ", " << poseAway2.theta << endl;

    // Calculate the ball position
    getBallPose(imgHsv, pink, poseBall);

    // Publish positions
    home1_pub.publish(poseHome1);
    home2_pub.publish(poseHome2);
    away1_pub.publish(poseAway1);
    away2_pub.publish(poseAway2);

    ball_pub.publish(poseBall);

}

// TODO: Dallon, try background subtraction!
void getCenter(Mat frame)
{
    Mat imgHsv;
    Mat gray;
    Mat imgGray;
    Mat gray2;

    cvtColor(frame, gray2, COLOR_BGR2GRAY);

    vector<Vec4f> lines_std2;

    dilate(gray2, gray2, getStructuringElement(MORPH_RECT, Size(20, 20)));

    // Use an to ignore the ceiling
    // I cropped the least amount I could.
    // The first two parameters of rect should be the top left x and y values!
    Rect roi(30, 0, 790, 480);
    Mat roiImage = gray2(roi);

    ls2->detect(roiImage, lines_std2);
    Mat drawnLines2(roiImage);

    //dilate(drawnLines2, drawnLines2, getStructuringElement(MORPH_RECT, Size(5, 5)));
    dilate(drawnLines2, drawnLines2, getStructuringElement(MORPH_RECT, Size(10, 10)));

    // Undo the dilation a little
    erode(drawnLines2, drawnLines2, getStructuringElement(MORPH_RECT, Size(4, 4)));

    // Sort lines into vertical and horizontal groups
    vector<Vec4f> v_lines; // (x,y) (x, y)
    vector<Vec4f> h_lines;
    vector<Vec4f> d_lines;

    float x1, x2, y1, y2;

    for (int i = 0; i < lines_std2.size(); i++)
    {
        // if the x's are about the same, it's vertical
        if (abs(lines_std2[i][0] - lines_std2[i][2]) < (abs(lines_std2[i][1] - lines_std2[i][3])/30))
        {
            v_lines.push_back(lines_std2[i]);
            x1 = v_lines[i][0];
            y1 = v_lines[i][1];
            x2 = v_lines[i][2];
            y2 = v_lines[i][3];
        }

        // if the y's are about the same, it's horizontal
        if (abs(lines_std2[i][1] - lines_std2[i][3]) < (abs(lines_std2[i][0] - lines_std2[i][2])/50))
        {
            double length = lines_std2[i][0] - lines_std2[i][2];
            //cout << length << endl;
            h_lines.push_back(lines_std2[i]);
        }
    }

    ls2->drawSegments(drawnLines2, lines_std2);

        //dilate(drawnLines2, drawnLines2, getStructuringElement(MORPH_RECT, Size(8, 2)));


    int leftCount = 0;
    float leftSum = 0;
    int rightCount = 0;
    float rightSum = 0;
    int topCount = 0;
    float topSum = 0;
    int bottomCount = 0;
    float bottomSum = 0;

    // x < 340 => left side
    // x > 420 => right side
    for (int i = 0; i < v_lines.size(); i++)
    {
        if (v_lines[i][0] < 340)
        {
            leftSum += v_lines[i][0];
            leftCount++;
        }
        else if (v_lines[i][0] > 420)
        {
            rightSum += v_lines[i][0];
            rightCount++;
        }
    }

    // lines with y < 30 is top
    // lines with y > 420 is bottom
    for (int i = 0; i < h_lines.size(); i++)
    {
        if (h_lines[i][1] < 30)
        {
            topSum += h_lines[i][1];
            topCount++;
        }
        else if (h_lines[i][1] > 420)
        {
            bottomSum += h_lines[i][1];
            bottomCount++;
        }
    }
    
    topYBorder = topSum/topCount;
    bottomYBorder = bottomSum/bottomCount;

    leftXBorder = leftSum/leftCount;

    rightXBorder = rightSum/rightCount;

    imshow("lines", drawnLines2);
    waitKey(60);


    // Using border values, crop the image.
    // The field is roughly 600 pixels wide and 415 pixels tall
    // The first two parameters of rect should be the top left x and y values!
    Rect roi2(leftXBorder, topYBorder, rightXBorder - leftXBorder, bottomYBorder - topYBorder);

    Mat roiImage2 = drawnLines2(roi2);

    // calculate the center using the ROI
    float centerY = (bottomYBorder - topYBorder)/2;
    float centerX = (rightXBorder - leftXBorder)/2;

    center_field.x = centerX;
    center_field.y = centerY;

    cout << "Center: " << center_field.x << ", " << center_field.y << endl;

    Point2d center_point (centerX, centerY);

    centerToWorldCoordinates(center_point);
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
             getCenter(frame);
        }
        //getCenter(frame);
       // processImage(frame);

        // Show the cropped field
        Rect roi(30, 0, 790, 480);
        Mat roiFrame1 = frame(roi);

        Rect roi2(leftXBorder + 4, topYBorder + 6, rightXBorder - leftXBorder, bottomYBorder - topYBorder);
        Mat roiFrame2 = roiFrame1(roi2);

        processImage(roiFrame2);


        firstRun = 1;

        imshow("Cropped Frame", roiFrame2);
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
    //namedWindow(LINES_WINDOW, 0);

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
    /*
    createTrackbar( "H_MIN", "Pre-Morphological", &H_MIN, 179, on_trackbar );
    createTrackbar( "H_MAX", "Pre-Morphological", &H_MAX, 179, on_trackbar );
    createTrackbar( "S_MIN", "Pre-Morphological", &S_MIN, 255, on_trackbar );
    createTrackbar( "S_MAX", "Pre-Morphological", &S_MAX, 255, on_trackbar );
    createTrackbar( "V_MIN", "Pre-Morphological", &V_MIN, 255, on_trackbar );
    createTrackbar( "V_MAX", "Pre-Morphological", &V_MAX, 255, on_trackbar );
    */
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
    home2_pub = nh.advertise<geometry_msgs::Pose2D>("truffle/home2", 5);
    away1_pub = nh.advertise<geometry_msgs::Pose2D>("truffle/away1", 5);
    away2_pub = nh.advertise<geometry_msgs::Pose2D>("truffle/away2", 5);
    ball_pub = nh.advertise<geometry_msgs::Pose2D>("truffle/ball", 5);

    ros::spin();
    return 0;
}