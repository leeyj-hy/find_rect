#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
using namespace ros;
using namespace std;
using namespace cv;


/*
//rectangle find & labeling
void setLabel(Mat& img, const vector<Point>& pts, const String& label)
{
    Rect rc = boundingRect(pts);
    rectangle(img, rc, Scalar(0, 0, 255), 1);
    putText(img, label, rc.tl(), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255));

}
//void setLabel(Mat& img, const vector<Point>& pts, const String& label)
//{
    //RotatedRect rc = minAreaRect(pts);
    //rc.center
    //rectangle(img, )
    //rectangle(img, rc, Scalar(0, 0, 255), 1);
    //putText(img, label, rc.tl(), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255));

//} 

int main(int argc, char* argv[])
{

    ros::init(argc, argv, "rect_find");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    cv::VideoCapture cap(2);

    if(!cap.isOpened())
    {
        ROS_INFO("Can't open the camera /n");
        return -1;
    }


    while (ros::ok())
    {
        Mat img ;
        cap >> img;

        if (img.empty())
        {
            cerr<<"Image load failed!" << endl;
            return -1;
        }

        Mat gray;
        cvtColor(img, gray, COLOR_BGR2GRAY);
        blur(gray, gray, Size(3,3));

        Mat bin;
        threshold(gray, bin, 200, 255, THRESH_BINARY_INV|THRESH_OTSU);
        imshow("bin", bin);
        vector<vector<Point>> contours;
        findContours(bin, contours, RETR_EXTERNAL,CHAIN_APPROX_NONE);

        for (vector<Point>& pts : contours)
        {
            if(contourArea(pts) < 1000)
                continue;

            vector<Point> approx;
            approxPolyDP(pts, approx, arcLength(pts, true)*0.06, true);

            int vtc = (int)approx.size();

            if(vtc == 3)
            {
                setLabel(img, pts, "TRI");
            }
            else if(vtc == 4)
            {
                setLabel(img, pts, "RECT");
            }
            else if(vtc > 4)
            {
                double len = arcLength(pts, true);
                double area = contourArea(pts);
                double ratio = 4. *CV_PI * area / (len*len);

                if(ratio > 0.8)
                {
                    setLabel(img, pts, "CIR");
                }
            }
        }

        imshow("img",img);
        if(waitKey(1)==3)
            break;
    }
    return 0;
}
*/









































/* 
//canny edge detection
int main(int argc, char **argv)
{
    ros::init(argc, argv, "rect_find");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    cv::VideoCapture cap(2);

    if(!cap.isOpened())
    {
        ROS_INFO("Can't open the camera /n");
        return -1;
    }

    Mat img_org;
    Mat img_gry;
    Mat img_cny;
    Mat img_rct;

    while(ros::ok())
    {
        cap>>img_org;
        cvtColor(img_org, img_gry, COLOR_BGR2GRAY);
        blur(img_gry, img_gry, Size(3,3));
        Canny(img_gry, img_cny, 150, 200);

        loop_rate.sleep();
        imshow("output", img_cny);

        if(waitKey(1)==3)
            break;
    }

    return 0; 
} */


























/*

//color(hue) recognization with trackbar
int lower_hue = 40, upper_hue = 80;
Mat src, src_hsv, mask;

void on_hue_changed(int, void*);

int main(int argc, char*argv[])
{

    ros::init(argc, argv, "rect_find");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    cv::VideoCapture cap(2);

    if(!cap.isOpened())
    {
        ROS_INFO("Can't open the camera /n");
        return -1;
    }


   

    
    while(ros::ok())
    {
        cap >> src;

        if(src.empty())
        {
            cerr<< "Image Load failed" << endl;
            return -1;
        }
        cvtColor(src, src_hsv, COLOR_BGR2HSV);

        imshow("src", src);

        namedWindow("mask");
        createTrackbar("LowHue", "mask", &lower_hue, 179, on_hue_changed);
        createTrackbar("Upper Hue", "mask", &upper_hue, 179, on_hue_changed);
        on_hue_changed(0,0);

      if(waitKey(1)==3)
            break;
    }
    return 0;
}

void on_hue_changed(int, void*)
{
    Scalar lowerb(lower_hue, 100, 0);
    Scalar upperb(upper_hue, 255, 255);
    inRange(src_hsv, lowerb, upperb, mask);

    imshow("mask", mask);
}

*/





//canny edge detection with trackbar

/* 
Mat src_gray;
Mat dst, detected_edges;
Mat src;
int lowThreshold = 50;
int highThreshold = 150;


static void CannyThreshold(int, void*)
{
	blur(src_gray, detected_edges, Size(3, 3));
	Canny(detected_edges, detected_edges, lowThreshold, highThreshold, 3);

	imshow("Canny Edge", detected_edges);
}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "rect_find");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    cv::VideoCapture cap(2);

    if(!cap.isOpened())
    {
        ROS_INFO("Can't open the camera /n");
        return -1;
    }
    
    while(ros::ok())
    {
        cap >> src;
        imshow("original img", src);

        if (src.empty())
        {
            std::cout << "Could not open or find the image!\n" << std::endl;
            std::cout << "Usage: " << argv[0] << " <Input image>" << std::endl;
            return -1;
        }


        cvtColor(src, src_gray, COLOR_BGR2GRAY);

        namedWindow("Canny Edge", WINDOW_AUTOSIZE);
        createTrackbar("Min Threshold:", "Canny Edge", &lowThreshold, 1000, CannyThreshold);
        createTrackbar("Max Threshold:", "Canny Edge", &highThreshold, 1000, CannyThreshold);
        CannyThreshold(0, 0);
        
        if(waitKey(1)==3)
                break;

    }
	return 0;
}
 */





Mat src_gray;
Mat dst, detected_edges;
Mat src;
int lowThreshold = 200;
int highThreshold = 255;


void setLabel(Mat& img, const vector<Point>& pts, const String& label)
{
    Rect rc = boundingRect(pts);
    rectangle(img, rc, Scalar(0, 0, 255), 1);
    putText(img, label, rc.tl(), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255));

}

static void CannyThreshold(int, void*)
{
	blur(src_gray, detected_edges, Size(3, 3));
	Canny(detected_edges, detected_edges, lowThreshold, highThreshold, 3);

	imshow("Canny Edge", detected_edges);
}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "rect_find");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    cv::VideoCapture cap(2);

    if(!cap.isOpened())
    {
        ROS_INFO("Can't open the camera /n");
        return -1;
    }
    
    while(ros::ok())
    {
        cap >> src;
        //imshow("original img", src);

        if (src.empty())
        {
            std::cout << "Could not open or find the image!\n" << std::endl;
            std::cout << "Usage: " << argv[0] << " <Input image>" << std::endl;
            return -1;
        }


        cvtColor(src, src_gray, COLOR_BGR2GRAY);

        //namedWindow("Canny Edge", WINDOW_AUTOSIZE);
        //createTrackbar("Min Threshold:", "Canny Edge", &lowThreshold, 1000, CannyThreshold);
        //createTrackbar("Max Threshold:", "Canny Edge", &highThreshold, 1000, CannyThreshold);
        //CannyThreshold(0, 0);
        blur(src_gray, detected_edges, Size(3, 3));


        //Mat gray;
        //cvtColor(img, gray, COLOR_BGR2GRAY);
        //blur(gray, gray, Size(3,3));

        Mat bin;
        
        createTrackbar("Arg 1: ", "bin", &lowThreshold, 1000);
        createTrackbar("Arg 2: ", "bin", &highThreshold, 255);
        
        threshold(detected_edges, bin, lowThreshold, highThreshold, THRESH_BINARY_INV|THRESH_OTSU);

        imshow("bin", bin);
        vector<vector<Point>> contours;
        findContours(bin, contours, RETR_EXTERNAL,CHAIN_APPROX_NONE);

        
        for (vector<Point>& pts : contours)
        {
            if(contourArea(pts) < 800)
                continue;

            vector<Point> approx;
            approxPolyDP(pts, approx, arcLength(pts, true)*0.06, true);

            int vtc = (int)approx.size();

            if(vtc == 3)
            {
                setLabel(src, pts, "TRI");
            }
            else if(vtc == 4)
            {
                setLabel(src, pts, "RECT");
            }
            else if(vtc > 4)
            {
                double len = arcLength(pts, true);
                double area = contourArea(pts);
                double ratio = 4. *CV_PI * area / (len*len);

                if(ratio > 0.8)
                {
                    setLabel(src, pts, "CIR");
                }
            }
        }   
        
        imshow("img",src);




        if(waitKey(1)==3)
                break;

    }
	return 0;
}





















