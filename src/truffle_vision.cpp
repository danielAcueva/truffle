#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream> 
#include <stdlib.h> // abs
#include <time.h>

using namespace std;
using namespace cv;
using namespace geometry_msgs;

#define FIELD_WIDTH 3.38
#define FIELD_HEIGHT 2.22
#define ROBOT_RADIUS 0.10
#define GUI_NAME "Camera"
#define DEBUG 1 // 1 for extra debug print statements

double field_width_pixels = 0; // goal to goal, normally about 643
double field_height_pixels = 0; // wall to wall, normally about 427

bool newKey = false;
void updateTrackbarValues();

int counter = 0;

// Jersey colors

// Home Jersey Colors
Scalar blue[] = {Scalar(89, 124, 225), Scalar(105, 172, 248)}; 
//Scalar purple[] = {Scalar(140, 26, 220), Scalar(154, 50, 240)};
Scalar purple[] = {Scalar(140, 26, 220), Scalar(161, 65, 244)};


// Away Jersey Colors
Scalar red[] = {Scalar(5, 82, 220), Scalar(15, 113, 227)}; // first scalar is low, second is high
Scalar yellow[] = {Scalar(0, 0, 228), Scalar(5, 6, 237)};

// Ball color
Scalar pink[] = {Scalar(168, 65, 213), Scalar(179, 86, 255)};

Scalar * colorPtr; // global pointer used to change color values

// default these values to blue or whatever
int H_MIN = 0;
int H_MAX = 179;
int S_MIN = 0;
int S_MAX = 255;
int V_MIN = 0;
int V_MAX = 255;

Point2d center_field;

Mat img;
int firstRun = 0;
bool cropped = false; // if 0, haven't cropped yet
bool cropping;
char lastKeyPressed;
Point refPt[2];

// robot vision thresholding
int thresh_val_home1 = 0;
int thresh_val_home2 = 0;
int thresh_val_away1 = 0;
int thresh_val_away2 = 0;

// roi_x and roi_y = top left
float roi_x, roi_y, roi_width, roi_height;

// Handlers for vision position publishers
ros::Publisher home1_pub;
ros::Publisher home2_pub;
ros::Publisher away1_pub;
ros::Publisher away2_pub;
ros::Publisher ball_pub;

// Use variables to store position of objects. These variables are very
// useful when the ball cannot be seen, otherwise we'll get the position (0, 0)
Pose2D poseHome1;
Pose2D poseHome2;
Pose2D poseAway1;
Pose2D poseAway2;
Pose2D poseBall;

void thresholdImage(Mat & imgHSV, Mat & imgGray, Scalar color[]) {

    inRange(imgHSV, color[0], color[1], imgGray);

    if (color == pink) {
        dilate(imgGray, imgGray, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));

    } else {
        dilate(imgGray, imgGray, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    }
}

Point2d getCenterOfMass(Moments moment) {
    double m10 = moment.m10;
    double m01 = moment.m01;
    double mass = moment.m00;
    double x = m10 / mass;
    double y = m01 / mass;
    return Point2d(x, y);
}

bool compareMomentAreas(Moments moment1, Moments moment2) {
    double area1 = moment1.m00;
    double area2 = moment2.m00;
    return area1 < area2;
}

Point2d imageToWorldCoordinates(Point2d point_i) {
    Point2d center_w = (point_i - center_field);

    // You have to split up the pixel to meter conversion
    // because it is a rect, not a square!
    center_w.x *= (FIELD_WIDTH/field_width_pixels); 
    center_w.y *= (FIELD_HEIGHT/field_height_pixels);

    // Reflect y
    center_w.y = -center_w.y;

    return center_w;
}

Point2d centerToWorldCoordinates(Point2d point_i) {
    Point2d center_w = (point_i - center_field);

    // You have to split up the pixel to meter conversion
    // because it is a rect, not a square!
    center_w.x *= (FIELD_WIDTH/field_width_pixels);
    center_w.y *= (FIELD_HEIGHT/field_height_pixels);

    // Reflect y
    center_w.y = -center_w.y;

    return center_w;
}


/*
* Sorts contours by area (descending order)
*/
vector<vector<Point>> sortContours(vector<vector<Point>> contours, vector<Vec4i> hierarchy)
{
    auto sortRuleLambda = [] (const vector<Point>& c1, const vector<Point>& c2) -> bool
    {
       return contourArea(c1, false) > contourArea(c2, false);
    };

    sort(contours.begin(), contours.end(), sortRuleLambda);

    return contours;
}


void getRobotPose(Mat &imgHsv, Scalar color[], Pose2D &robotPose) {
    Mat imgGray;

    //imgHsv and imgGray are passed by reference
    thresholdImage(imgHsv, imgGray, color);

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

    if (&robotPose == & poseHome1) {

        // time test
        //robotPose.x = -1;
       // robotPose.y = -1;
      //  robotPose.theta = -1;


        if (!firstRun) {
            thresh_val_home1 = threshold(imgGray, th3, 0, 255, THRESH_BINARY | THRESH_OTSU);
        } else {
            threshold(imgGray, th3, thresh_val_home1, 255, THRESH_BINARY);
        }

        if (lastKeyPressed == 'd') {
            colorPtr = blue;
            updateTrackbarValues();
            imshow(GUI_NAME, th3);
        }

    } else if (&robotPose == &poseHome2) {
        if (!firstRun) {
            thresh_val_home2 = threshold(imgGray, th3, 0, 255, THRESH_BINARY | THRESH_OTSU);
        } else {
            threshold(imgGray, th3, thresh_val_home2, 255, THRESH_BINARY);
        }

        if (lastKeyPressed == 'e') {
            colorPtr = purple;
            updateTrackbarValues();
            imshow(GUI_NAME, th3);
        }

    } else if (&robotPose == &poseAway1) {
        if (!firstRun) {
            thresh_val_away1 = threshold(imgGray, th3, 0, 255, THRESH_BINARY | THRESH_OTSU);
        } else {
            threshold(imgGray, th3, thresh_val_away1, 255, THRESH_BINARY);
        }

        if (lastKeyPressed == 'f') {
            colorPtr = red;
            updateTrackbarValues();
            imshow(GUI_NAME, th3);
        }

    } else if (&robotPose == &poseAway2) {
        if (!firstRun) {
            thresh_val_away2 = threshold(imgGray, th3, 0, 255, THRESH_BINARY | THRESH_OTSU);
        } else {
            threshold(imgGray, th3, thresh_val_away2, 255, THRESH_BINARY);
        }

        if (lastKeyPressed == 'g') {
            colorPtr = yellow;
            updateTrackbarValues();
            imshow(GUI_NAME, th3);
        }

    }

    vector<vector<Point>> contours;
    vector<Moments> mm;
    vector<Vec4i> hierarchy;

    //Find countour, fill up the vector
    findContours(th3, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    //Heirarchy shows how many objects are found.
    // We want to make sure there are two
    // we have two colored identifiers on the robot jersey
    if (hierarchy.size() != 2) {

        if (hierarchy.size() < 2)
        {
            if (DEBUG) {
                switch (lastKeyPressed) {
                    case 'd':
                        if (&robotPose == &poseHome1)
                        {
                            cout << "[Vision] Robot Blue - Bad hierarchy" << endl;
                        }
                        break;
                    case 'e':
                        if (&robotPose == &poseHome2)
                        {
                            cout << "[Vision] Robot Purple - Bad hierarchy" << endl;
                        }
                        break;
                    case 'f':
                        if (&robotPose == &poseAway1)
                        {
                          cout << "[Vision] Robot Red - Bad hierarchy" << endl;
                        }
                        break;
                    case 'g':
                        if (&robotPose == &poseAway2)
                        {
                            cout << "[Vision] Robot Yellow - Bad hierarchy" << endl;
                        }
                        break;
                    default:
                        break;
                }
            }
            return;
        }

        // Delete all contours except two largest
        // This is an attempt to get rid of small contours that show up that we don't want
        contours = sortContours(contours, hierarchy); // sort by area (descending)
        contours.resize(2); // keep only first 2 elements of vector
    }

    //Add to vector of moments, the heirarchy data
    for (int i = 0; i < contours.size(); i++)
        mm.push_back(moments((Mat) contours[i]));

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
    angle = angle * 180 / M_PI;
    robotPose.x = robotCenter.x;
    robotPose.y = robotCenter.y;
    robotPose.theta = angle;

    if (DEBUG) {
        switch (lastKeyPressed) {
        case 'd':
            if (&robotPose == &poseHome1)
            {
                cout << "[Vision] Robot Blue: " << robotPose.x << ", " << robotPose.y << ", " << robotPose.theta << endl;
            }
            break;
        case 'e':
            if (&robotPose == &poseHome2)
            {
                cout << "[Vision] Robot Purple: " << robotPose.x << ", " << robotPose.y << ", " << robotPose.theta << endl;
            }
            break;
        case 'f':
            if (&robotPose == &poseAway1)
            {
              cout << "[Vision] Robot Red: " << robotPose.x << ", " << robotPose.y << ", " << robotPose.theta << endl;
            }
            break;
        case 'g':
            if (&robotPose == &poseAway2)
            {
            cout << "[Vision] Robot Yellow: " << robotPose.x << ", " << robotPose.y << ", " << robotPose.theta << endl;
            }
            break;
        default:
            break;
        }
    }

}

void getBallPose(Mat & imgHsv, Scalar color[], geometry_msgs::Pose2D & ballPose) {
    Mat imgGray;
    thresholdImage(imgHsv, imgGray, color);


    // Show the ball if C was last pressed
    if (lastKeyPressed == 'c') {
        imshow(GUI_NAME, imgGray);
        colorPtr = pink;
        updateTrackbarValues();
    }

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(imgGray, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    if (hierarchy.size() != 1) {
        if (DEBUG) {
            //cout << "[Vision] ERROR: Ball hierarchy failed";
        }
        return;
    }

    Moments mm = moments((Mat) contours[0]);
    Point2d ballCenter = imageToWorldCoordinates(getCenterOfMass(mm));

    ballPose.x = ballCenter.x; //Center of field offset
    ballPose.y = ballCenter.y; //Center of field offset
    ballPose.theta = 0;

    if (lastKeyPressed == 'c' && DEBUG) {
        cout << "[Vision] Ball: " << ballPose.x << ", " << ballPose.y << endl;
    }
}

void processImage(Mat frame) {
    //Create a Mat variable imgHsv
    Mat imgHsv;
    Mat gray;
    cvtColor(frame, imgHsv, COLOR_BGR2HSV);

    //Calculate the robot position. imgHsv and poseHome1 are passed by reference
    getRobotPose(imgHsv, blue, poseHome1);
    getRobotPose(imgHsv, purple, poseHome2);
    getRobotPose(imgHsv, red, poseAway1);
    getRobotPose(imgHsv, yellow, poseAway2);

    // Calculate the ball position
    getBallPose(imgHsv, pink, poseBall);

  //  time_t          s;  // Seconds
   // struct timespec spec;

    // Publish positions
    
    /*
    if (counter > 20)
    {
        counter = 0;

        clock_gettime(CLOCK_REALTIME, &spec);

        s  = spec.tv_sec;
        ms = round(spec.tv_nsec / 1.0e6); // Convert nanoseconds to milliseconds
        printf("Current time: .%03ld seconds since the Epoch\n",  (intmax_t)s);
    }
    else
    {
        
        counter++;
    }

*/

    home1_pub.publish(poseHome1);
    home2_pub.publish(poseHome2);
    away1_pub.publish(poseAway1);
    away2_pub.publish(poseAway2);

    ball_pub.publish(poseBall);

}

void getCenter(Mat frame) {
    Mat bgImage;
    Mat imgHsv;
    Mat gray;
    Mat imgGray;
    Mat gray2;

    // background subtraction example below
    //absdiff(bgImage,frame,frame); // Absolute differences between the 2 images 
    //threshold(frame,frame,15,255,CV_THRESH_BINARY); // set threshold to ignore small differences you can also use inrange function

    // calculate the center using the ROI
    float centerY = abs(refPt[1].y - refPt[0].y) / 2;
    float centerX = abs(refPt[1].x - refPt[0].x) / 2;

    center_field.x = centerX;
    center_field.y = centerY;

    if (DEBUG) {
        cout << "Center: " << center_field.x << ", " << center_field.y << endl;
    }

    Point2d center_point(centerX, centerY);

    centerToWorldCoordinates(center_point);
}

//Called when data is subscribed
void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    Mat result;
    try {
        //capture the data
        Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        img = frame;
        //Process the image. This will process the data

        if (!cropped) {
            imshow(GUI_NAME, frame);
            waitKey(60);
            // Wait for user to crop
            // And calculate center
        } else {
            if (!firstRun) {
                getCenter(frame);
            }

            // Crop the field
            Rect roi(roi_x, roi_y, roi_width, roi_height);
            Mat roiFrame = frame(roi);

            // update field_width_pixels and field_width_height
            field_width_pixels = roi_width;
            field_height_pixels = roi_height;

            processImage(roiFrame);

            firstRun = 1;

            if (lastKeyPressed == 'a') {
                imshow(GUI_NAME, frame);
            } else if (lastKeyPressed == 'b') // show cropped frame
            {
                imshow(GUI_NAME, roiFrame);
            }
        }

        // Wait for key press (for the display menu)
        char key = waitKey(60);
        if (key != lastKeyPressed && key != -1) {
            newKey = true;
        } else {
            newKey = false;
        }
        if (key != -1) {
            lastKeyPressed = key;
        }

        if (key == 'q') {
            ros::shutdown();
        }

    } catch (cv_bridge::Exception & e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void mouseCallback(int event, int x, int y, int flags, void * userdata) {
    static bool mouse_left_down = false;

    // if the left mouse button was clicked, record the starting
    // (x, y) coordinates and indicate that cropping is being
    // performed
    if (!cropped) {
        if (event == EVENT_LBUTTONDOWN) {
            Point point1;
            point1.x = x;
            point1.y = y;

            refPt[0] = point1;
            cropping = true;
        } else if (cropping && event != EVENT_LBUTTONUP) {
            Point currPoint;
            currPoint.x = x;
            currPoint.y = y;
            rectangle(img, refPt[0], currPoint, (0, 255, 0), 2);

            imshow(GUI_NAME, img);

        } else if (event == EVENT_LBUTTONUP) // check to see if the left mouse button was released
        {
            // record the ending (x, y) coordinates and indicate that
            // the cropping operation is finished
            Point point2;
            point2.x = x;
            point2.y = y;

            refPt[1] = point2;
            cropping = false;


            if (refPt[0].x < refPt[1].x) // user dragged from left like a normal person
            {
                roi_x = refPt[0].x;
                roi_y = refPt[0].y;
            }
            else // user is weird and dragged from right
            {
                roi_x = refPt[1].x;
                roi_y = refPt[1].y;
            }

            roi_width = abs(refPt[1].x - refPt[0].x);
            roi_height = abs(refPt[1].y - refPt[0].y);

            try {
                Rect roi(roi_x, roi_y, roi_width, roi_height);
                Mat roiFrame = img(roi);

                // replace window with cropped img
                imshow(GUI_NAME, roiFrame);
                cropped = true;
            } catch (cv::Exception & e) {
                cout << "Exception: Bad crop." << endl;
            }

        }
    }

}

// This function is called whenever a trackbar changes
void on_trackbar(int, void * ) {

    Scalar scalar1 = Scalar(H_MIN, S_MIN, V_MIN);
    Scalar scalar2 = Scalar(H_MAX, S_MAX, V_MAX);

    // blue
    colorPtr[0] = scalar1;
    colorPtr[1] = scalar2;
}

void createHSVTrackbars() {

    //create trackbars and insert them into window
    //3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
    //the max value the trackbar can move (eg. H_HIGH),
    //and the function that is called whenever the trackbar is moved(eg. on_trackbar)
    createTrackbar("H_MIN", GUI_NAME, & H_MIN, 179, on_trackbar);
    createTrackbar("H_MAX", GUI_NAME, & H_MAX, 179, on_trackbar);
    createTrackbar("S_MIN", GUI_NAME, & S_MIN, 255, on_trackbar);
    createTrackbar("S_MAX", GUI_NAME, & S_MAX, 255, on_trackbar);
    createTrackbar("V_MIN", GUI_NAME, & V_MIN, 255, on_trackbar);
    createTrackbar("V_MAX", GUI_NAME, & V_MAX, 255, on_trackbar);
}

void updateTrackbarValues() {

    if (newKey) // if new key, update trackbar 
    {
        H_MIN = colorPtr[0](0);
        S_MIN = colorPtr[0](1);
        V_MIN = colorPtr[0](2);

        H_MAX = colorPtr[1](0);
        S_MAX = colorPtr[1](1);
        V_MAX = colorPtr[1](2);

        setTrackbarPos("H_MIN", GUI_NAME, colorPtr[0](0));
        setTrackbarPos("S_MIN", GUI_NAME, colorPtr[0](1));
        setTrackbarPos("V_MIN", GUI_NAME, colorPtr[0](2));

        setTrackbarPos("H_MAX", GUI_NAME, colorPtr[1](0));
        setTrackbarPos("S_MAX", GUI_NAME, colorPtr[1](1));
        setTrackbarPos("V_MAX", GUI_NAME, colorPtr[1](2));
    }

}

void printMenu() {
    printf("Menu: Make sure the 'Display' Window is in focus to type\n");
    printf("a - Show original image\n");
    printf("b - Show cropped image\n");

    cout << "c - Show ball position" << endl;
    cout << "d - Show ally1 position" << endl;
    cout << "e - Show ally2 position" << endl;
    cout << "f - Show opp1 position" << endl;
    cout << "g - show opp2 position" << endl;

    printf("e - Show borders and ball location\n");
    printf("z - Modify borders of field\n");
    printf("q - Quit\n");
}

int main(int argc, char * * argv) {

    cropping = false;
    colorPtr = blue; // default pointer value

    ros::init(argc, argv, "vision_sim");
    ros::NodeHandle nh;

    ros::NodeHandle priv_nh("~"); //Create private node handle

    // Create OpenCV Window and add a mouse callback for clicking
    namedWindow(GUI_NAME, CV_WINDOW_AUTOSIZE);
    setMouseCallback(GUI_NAME, mouseCallback, NULL);
    printMenu();

    createHSVTrackbars();

    cout << "ATTENTION: You must first CROP the field" << endl;

    // Subscribe to camera
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe("/usb_cam_away/image_raw", 1, imageCallback);

    // Publish the robot's position (x and y)
    home1_pub = nh.advertise < geometry_msgs::Pose2D > ("home1", 5);
    home2_pub = nh.advertise < geometry_msgs::Pose2D > ("home2", 5);
    away1_pub = nh.advertise < geometry_msgs::Pose2D > ("away1", 5);
    away2_pub = nh.advertise < geometry_msgs::Pose2D > ("away2", 5);
    ball_pub = nh.advertise < geometry_msgs::Pose2D > ("ball", 5);

    ros::spin();
    return 0;
}