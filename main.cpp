/**
 * @file main.cpp.
 *
 * @brief Implements the main file for autonomous object tracking and following.
 *        
 * This file is where the actual object tracking and following with the Parrot AR.Drone 2.0.
 */

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

#define WIDTH  640
#define HEIGHT 360
#define STAT_REFRESH_RATE 15

int stat_count = 1;
int follow = 0;
int auto_land = 0;
int camera = 0;
int detected = 0;

int pos_x = 0;
int pos_y = 0;
double area;

int deadzone_x = 320;
int deadzone_y = 180;
int min_area = 0;
int max_area = 500;
int hue_low = 0;
int hue_high = 179;
int sat_low = 0;
int sat_high = 255;
int val_low = 0;
int val_high = 255;

double vx;
double vy;
double vz;
double vr;

using namespace cv;
using namespace std;

Mat stat_image(HEIGHT, WIDTH, CV_8UC3, Scalar(0, 0, 0));
Mat deadzone_image(HEIGHT, WIDTH, CV_8UC3, Scalar(0, 0, 0));
Mat contour_image(HEIGHT, WIDTH, CV_8UC3, Scalar(0, 0, 0));

ARDrone ardrone;

/**
 * @brief Prints the keyboard key functions to the console. 
 *
 * This function simply prints the keyboard keys with their function to the console.
 */

void consoleStrings()
{
    cout << "------- KEYBOARD FUNCTIONS -------" << endl;
    cout << "W: Move forward" << endl;
    cout << "S: Move backward" << endl;
    cout << "A: Rotate left" << endl;
    cout << "D: Rotate right" << endl;
    cout << "Q: Move left" << endl;
    cout << "A: Move right" << endl;
    cout << "I: Move up" << endl;
    cout << "K: Move down" << endl;
    cout << "C: Camera" << endl;
    cout << "V: Calibrate" << endl;
    cout << "T: Flat trim" << endl;
    cout << "P: Emergency" << endl;
    cout << "F: Follow on/off" << endl;
    cout << "C: Switch camera" << endl;
    cout << "L: Land autonomously" << endl;
}

/**
 * @brief Prints the drone and object statistics to a given image
 *
 * This function prints several statistics about the detected object, the drone, and whether the
 * object should be followed or not.
 */

void drawStats(void)
{
    stat_image = Scalar(0, 0, 0);

    char area_str[40];
    char bat_str[40];

    int bat = 0;

    sprintf_s(area_str, "Object area: %.2f", (area / 100000));
    sprintf_s(bat_str, "Battery: %d %%", bat);

    if (detected)
    {
        putText(stat_image, "OBJECT DETECTED", Point(10, 30), FONT_HERSHEY_COMPLEX_SMALL, 0.9, Scalar(0, 255, 0));
    }
    else
    {
        putText(stat_image, "NO OBJECT DETECTED", Point(10, 30), FONT_HERSHEY_COMPLEX_SMALL, 0.9, Scalar(0, 0, 255));
    }

    if (follow)
    {
        putText(stat_image, "FOLLOWING ON", Point(10, 60), FONT_HERSHEY_COMPLEX_SMALL, 0.9, Scalar(0, 255, 0));
    }
    else
    {
        putText(stat_image, "FOLLOWING OFF", Point(10, 60), FONT_HERSHEY_COMPLEX_SMALL, 0.9, Scalar(0, 0, 255));
    }

    if (auto_land)
    {
        putText(stat_image, "AUTO LANDING ON", Point(10, 90), FONT_HERSHEY_COMPLEX_SMALL, 0.9, Scalar(0, 255, 0));
    }
    else
    {
        putText(stat_image, "AUTO LANDING OFF", Point(10, 90), FONT_HERSHEY_COMPLEX_SMALL, 0.9, Scalar(0, 0, 255));
    }

    putText(stat_image, area_str, Point(10, HEIGHT - 50), FONT_HERSHEY_COMPLEX_SMALL, 0.9, Scalar(255, 255, 0));
    putText(stat_image, bat_str, Point(10, HEIGHT - 20), FONT_HERSHEY_COMPLEX_SMALL, 0.9, Scalar(255, 255, 0));
}

void drawDeadzone(void)
{
    int x1 = (WIDTH / 2) - (deadzone_x / 2);
    int y1 = (HEIGHT / 2) - (deadzone_y / 2);
    int x2 = (WIDTH / 2) + (deadzone_x / 2);
    int y2 = (HEIGHT / 2) + (deadzone_y / 2);

    deadzone_image = Scalar(0, 0, 0);
    rectangle(deadzone_image, Point(x1, y1), Point(x2, y2), Scalar(255, 0, 255), 1);
}

void loadConfigValues()
{
    FileStorage fs;

    if (camera == 0)
    {
        fs.open("config/front_camera_config.xml", FileStorage::READ);
    }
    if (camera == 1)
    {
        fs.open("config/bottom_camera_config.xml", FileStorage::READ);
    }

    fs["HueLOW"] >> hue_low;
    fs["HueHIGH"] >> hue_high;
    fs["SaturationLOW"] >> sat_low;
    fs["SaturationHIGH"] >> sat_high;
    fs["ValueLOW"] >> val_low;
    fs["ValueHIGH"] >> val_high;
    fs["AreaMIN"] >> min_area;
    fs["AreaMAX"] >> max_area;
    fs["DeadzoneX"] >> deadzone_x;
    fs["DeadzoneY"] >> deadzone_y;

    fs.release();
}

void saveConfigValues()
{
    FileStorage fs;

    if (camera == 0)
    {
        fs.open("config/front_camera_config.xml", FileStorage::WRITE);
    }
    if (camera == 1)
    {
        fs.open("config/bottom_camera_config.xml", FileStorage::WRITE);
    }

    fs << "HueLOW" << hue_low;
    fs << "HueHIGH" << hue_high;
    fs << "SaturationLOW" << sat_low;
    fs << "SaturationHIGH" << sat_high;
    fs << "ValueLOW" << val_low;
    fs << "ValueHIGH" << val_high;
    fs << "AreaMIN" << min_area;
    fs << "AreaMAX" << max_area;
    fs << "DeadzoneX" << deadzone_x;
    fs << "DeadzoneY" << deadzone_y;

    fs.release();
}

/**
 * @brief Generates the "control window" for the object detection settings.
 *        
 * In this function the trackbars are being generated for the object detection parameters.
 * Those parameters are:
 * - Hue (low)  
 * - Hue (high)  
 * - Saturation (low)  
 * - Saturation (high)  
 * - Value (low)  
 * - Value (high)  
 * 
 * The values of those parameters depend on the used camera.
 */

void controlWindow()
{
    destroyWindow("Control");
    namedWindow("Control", WINDOW_NORMAL);
    resizeWindow("Control", 1280, 480);
    moveWindow("Control", 0, 395);

    createTrackbar("Hue LOW", "Control", &hue_low, 179);
    createTrackbar("Hue HIGH", "Control", &hue_high, 179);
    createTrackbar("Sat LOW", "Control", &sat_low, 255);
    createTrackbar("Sat HIGH", "Control", &sat_high, 255);
    createTrackbar("Val LOW", "Control", &val_low, 255);
    createTrackbar("Val HIGH", "Control", &val_high, 255);
    createTrackbar("Area MIN", "Control", &min_area, 500);
    createTrackbar("Area MAX", "Control", &max_area, 500);
}

/**
 * Assigns functions to keyboard key presses.
 *
 * @param key The key input with a small wait statement.
 */

void keyFunctions(int key)
{
    if (key == ' ')
    {
        if (ardrone.onGround()) ardrone.takeoff();
        else ardrone.landing();
    }
    if (key == 'c')
    {
        saveConfigValues();
        camera = !camera;
        loadConfigValues();
        controlWindow();
        drawDeadzone();
    }
    if (key == 'w') vx = 1.0;
    if (key == 's') vx = -1.0;
    if (key == 'a') vr = 1.0;
    if (key == 'd') vr = -1.0;
    if (key == 'q') vy = 1.0;
    if (key == 'e') vy = -1.0;
    if (key == 'i') vz = 1.0;
    if (key == 'k') vz = -1.0;
    if (key == 'v') ardrone.setCalibration();
    if (key == 't') ardrone.setFlatTrim();
    if (key == 'p') ardrone.emergency();
    if (key == 'f') follow = !follow;
    if (key == 'l') auto_land = !auto_land;
}

int detectObject(Mat img_hsv)
{
    Mat mask;
    Mat result(HEIGHT, WIDTH, CV_8U, Scalar(0));

    namedWindow("Mask", WINDOW_NORMAL);
    resizeWindow("Mask", WIDTH, HEIGHT);
    moveWindow("Mask", WIDTH, 0);

    inRange(img_hsv, Scalar(hue_low, sat_low, val_low), Scalar(hue_high, sat_high, val_high), mask);

    imshow("Mask", mask);

    Mat denoise = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    morphologyEx(mask, mask, cv::MORPH_CLOSE, denoise);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    int largest_contour_id = -1;
    double contour_max_area = 0.0;

    findContours(mask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point());

    for (int i = 0; i < contours.size(); i++)
    {
        if (contourArea(contours[i]) > contour_max_area)
        {
            contour_max_area = contourArea(contours[i]);
            largest_contour_id = i;
        }
    }

    contour_image = Scalar(0, 0, 0);

    drawContours(contour_image, contours, largest_contour_id, Scalar(0, 255, 0), 2, 8);
    drawContours(result, contours, largest_contour_id, Scalar(255), CV_FILLED, 8);

    Moments Moments = moments(result);

    double m01 = Moments.m01;
    double m10 = Moments.m10;
    area = Moments.m00;

    if (area > (min_area * 100000))
    {
        pos_x = m10 / area;
        pos_y = m01 / area;

        return 1;
    }

    return 0;
}

void followObject()
{
    switch (camera)
    {
        case 0: // Front camera

            //

            if (pos_y < (HEIGHT - deadzone_y) / 2 || pos_y > (HEIGHT + deadzone_y) / 2)
            {
                vz = (pos_y - (HEIGHT / 2)) / -250.0;
            }
            if (pos_x < (WIDTH - deadzone_x) / 2 || pos_x > (WIDTH + deadzone_x) / 2)
            {
                vr = (pos_x - (WIDTH / 2)) / -200.0;
            }
            if (area < max_area * 100000)
            {
                vx = 0.3;
            }
            if (area > (max_area + 10) * 100000)
            {
                vx = -0.3;
            }
            break;

        case 1: // Bottom camera
            if (pos_y < (HEIGHT - deadzone_y) / 2 || pos_y > (HEIGHT + deadzone_y) / 2)
            {
                vx = (pos_y - (HEIGHT / 2)) / -400.0;
            }

            if (pos_x < (WIDTH - deadzone_x) / 2 || pos_x > (WIDTH + deadzone_x) / 2)
            {
                vy = (pos_x - (WIDTH / 2)) / -800.0;
            }

            if (auto_land)
            {
                vz = -0.1;

                if (area > max_area * 100000)
                {
                    ardrone.landing();
                }
            }
            break;

        default:
            break;
    }
}

int main(int argc, char* argv[])
{
    VideoCapture cap(0);

    if (!cap.isOpened())
    {
        cout << "Failed to open capture device." << endl;
        return -1;
    }

    cap.set(CAP_PROP_FRAME_WIDTH, WIDTH);
    cap.set(CAP_PROP_FRAME_HEIGHT, HEIGHT);

    namedWindow("Input", WINDOW_NORMAL);
    resizeWindow("Input", WIDTH, HEIGHT);
    moveWindow("Input", 0, 0);

    consoleStrings();
    loadConfigValues();
    controlWindow();
    drawDeadzone();

    while (true)
    {
        int key = waitKey(33);
        if (key == 0x1b) break; // ESC     
        keyFunctions(key);

        Mat input_image;
        cap >> input_image;

        Mat input_image_hsv;

        cvtColor(input_image, input_image_hsv, COLOR_BGR2HSV);

        detected = detectObject(input_image_hsv);

        if (detected)
        {
            drawMarker(input_image, Point(pos_x, pos_y), Scalar(0, 255, 255), MARKER_CROSS, 25, 2);

            if (follow)
            {
                followObject();
            }
        }

        if (stat_count == STAT_REFRESH_RATE)
        {
            drawStats();
            stat_count = 1;
        }
        else
        {
            stat_count++;
        }

        input_image = input_image + stat_image + contour_image + deadzone_image;
        imshow("Input", input_image);

        msleep(30);
    }

    saveConfigValues();

    return 0;
}
