#include "hsvselector.h"
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


//Default constructor
HSVSelector::HSVSelector()
{
    std::cout << "You are in the HSVSelector constructor but with no matrix called. You may want to add an image" << '\n';
}


HSVSelector::HSVSelector(cv::Scalar lwBd, cv::Scalar upBd, bool dispWndw, cv::Scalar lwBd2, cv::Scalar upBd2, bool dispLns, bool tempLns)
{
    lowerBd = lwBd;
    upperBd = upBd;
    lowerBd2 = lwBd2;
    upperBd2 = upBd2;
    erosion_size = 5;
    // Determines if the output is shown in a window
    dispWindow = dispWndw;
    // Determines if a MomentFeature will be created, and if so whether lines will be temporary.
    dispLines = dispLns;
    tempLines = tempLns;

    if(dispWindow)
    {
        cv::namedWindow("HSVSelectorWindow");
    }
    if(dispLines)
    {
        MmFdr = MomentFinder(dispLines, tempLines);
    }

    element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),cv::Point(erosion_size, erosion_size) );
}


cv::Mat HSVSelector::newImage(cv::Mat &image)
{
    img = image;

    // Convert the image to hsv;
    cv::cvtColor(image, hsvImg, CV_BGR2HSV);

    // Threshold the image for the first time. If a second is required, the next code will take care of it.
    cv::Mat imgThresholded;
    cv::inRange(hsvImg, lowerBd, upperBd, imgThresholded);

    // If a second threshold is desired (to add a separate part of the spectrum), this code will handle that
    if (lowerBd2!=SCALAR_ZERO && upperBd2!=SCALAR_ZERO)
    {
        cv::Mat first_range, second_range;
        first_range = imgThresholded;
        cv::inRange(hsvImg,lowerBd2,upperBd2, second_range);
        imgThresholded.setTo(0);

        cv::addWeighted(first_range, 1.0, second_range, 1.0, 0.0, imgThresholded);
    }


    //morphological opening (removes small objects from the foreground)
    erode(imgThresholded, imgThresholded, element );
    dilate( imgThresholded, imgThresholded, element );

    // In the case we would like the lines displayed - this will update our image,
    // it will then display if requested, and finally return the image with lines.
    if(dispLines)
    {
        cv::Mat withLines = MmFdr.updateImg(imgThresholded);
        if(dispWindow) {cv::imshow("HSVSelectorWindow",withLines);}
        return withLines;
    }

    if(dispWindow)
    {
        cv::imshow("HSVSelectorWindow",image);
    }

    return imgThresholded;
}

void HSVSelector::modifyBounds1(cv::Scalar nwLwBd, cv::Scalar nwUpBd)
{
    upperBd = nwUpBd;
    lowerBd = nwLwBd;
}

void HSVSelector::modifyBounds2(cv::Scalar nwLwBd, cv::Scalar nwUpBd)
{
    upperBd2 = nwUpBd;
    lowerBd2 = nwLwBd;
}
