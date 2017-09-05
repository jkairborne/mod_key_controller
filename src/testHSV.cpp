#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

#include <algorithm>
#include <iostream>

#include "hsvselector.h"

static const std::string OPENCV_WINDOW = "Image1";


using std::vector;
using namespace cv;

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    HSVSelector yellow;
    Mat mask, element, imgLines;
    bool firstTime;


    // Function declarations
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

public:
    ImageConverter()
        : it_(nh_)
    {
    image_sub_ = it_.subscribe("/ardrone/image_raw", 1,&ImageConverter::imageCb, this);
    //    HSVSelector(cv::Scalar lwBd, cv::Scalar upBd, bool dispWndw = true, cv::Scalar lwBd2 = SCALAR_ZERO,
    //                cv::Scalar upBd2 = SCALAR_ZERO, bool dispLns = true, bool tempLns = true);

    yellow = HSVSelector(RED_LOWER,RED_UPPER,true,ORANGE_LOWER,ORANGE_UPPER);
    cv::namedWindow(OPENCV_WINDOW);

    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }
}; // End class

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");

    ImageConverter ic;

    ros::spin();
    return 0;
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(firstTime)
    {
        imgLines = Mat::zeros( cv_ptr->image.size(), CV_8UC3 );
        firstTime = false;
    }

    cv::Mat result = yellow.newImage(cv_ptr->image);
    //cv::imshow("Image1",result);
    cv::imshow("Image1",cv_ptr->image);
    cv::waitKey(3);


}// end ImageConverter::imageCb


/*
HSV color space is also consists of 3 matrices, HUE, SATURATION and VALUE. In OpenCV, value range for  HUE, SATURATION  and VALUE  are respectively 0-179, 0-255 and 0-255. HUE represents the color, SATURATION  represents the amount to which that respective color is mixed with white and VALUE  represents the  amount to which that respective color is mixed with black.

In the above application, I have considered that the red object has HUE, SATURATION and VALUE in between 170-180, 160-255, 60-255 respectively. Here the HUE is unique for that specific color distribution of that object. But SATURATION and VALUE may be vary according to the lighting condition of that environment.

Hue values of basic colors


These are approximate values. You have to find the exact range of HUE values according to the color of the object. I found that the range of 170-179 is perfect for the range of hue values of my object. The SATURATION and VALUE is depend on the lighting condition of the environment as well as the surface of the object.

How to find the exact range of HUE, SATURATION and VALUE for a object is discussed later in this post.
*/
