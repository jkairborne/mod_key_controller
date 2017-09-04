#include "momentfinder.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <algorithm>
#include <iostream>

MomentFinder::MomentFinder(bool modifyimage, bool tempmodify, int nLines)
{
    linesOn = modifyimage;
    linesTemp = tempmodify;
    numTempLines = nLines;
    iLastX = -1;
    iLastY = -1;
    posX = 0;
    posY = 0;
    firstImg = true;

    if(linesTemp) { lastPts.resize(numTempLines+1);}
}

// This function returns an image which will have been modified in the drawLines function
cv::Mat MomentFinder::updateImg(cv::Mat& updImg)
{
    // If this is the first time this MomentFinder has seen an image, we need to resize initialize the lines matrix.
    // This will ensure that we are able to add the matrices together at the end
    if(firstImg)
    {
        linesImg = cv::Mat(updImg.size(),updImg.type());
        firstImg = false;
    }
    img = updImg;
    calculateMoments();
    drawLine();
    return img;
}

    // This function calculates the moments of the image
void MomentFinder::calculateMoments()
{
    cv::Moments oMoments = cv::moments(img);

    dM01 = oMoments.m01;
    dM10 = oMoments.m10;
    dArea = oMoments.m00;
}

// This function draws the line following the central moment of the object.
void MomentFinder::drawLine()
{
    // if the area <= 100, I consider that the there are no object in the image and it's because of the noise, the area is not zero
    if (dArea > 100)
    {
     //calculate the central position of the pixels
     int posX = dM10 / dArea;
     int posY = dM01 / dArea;

     // Bounds checking
     if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
     {
         //if linesTemp is true, we want to keep only numTempLines points. We maintain a vector which will keep points in memory.
         if (linesTemp)
         {
             // Erase the last point to the list of points.
             //Notice that here we go from begin to numTempLines because we actually have one extra point!
             //In a "normal" loop we would need to add a -1.
             lastPts.erase(lastPts.begin() + numTempLines);

             // Add the latest point to the beginning of the list
             lastPts.insert(lastPts.begin(),cv::Point(posX,posY));

             // Reset the lines image to zero.
             linesImg.setTo(0);
             for (int i=0;i<numTempLines;i++)
             {
                 // For every pair of points, write out the line onto linesImg.
                 cv::line(linesImg, lastPts[i], lastPts[i+1], cv::Scalar(255,255,255), 2);
             }
         }
         else
         {
              //Draw a line from the previous point to the current point
              cv::line(linesImg, cv::Point(posX, posY), cv::Point(iLastX, iLastY), cv::Scalar(255,255,255), 2);
         }
     }
     //Update the last numbers for the next iteration.
     iLastX = posX;
     iLastY = posY;
    }

    // if linesOn is true, we would like to overwrite the images
    if(linesOn)
    {
        img = img + linesImg;
    }

    // The actual returning portion of this function is accomplished by the updateImage function.

} // end drawLine

cv::Point MomentFinder::getCentre()
{
    int centreX = dM10 / dArea;
    int centreY = dM01 / dArea;
    cv::Point centre = cv::Point(centreX,centreY);

    return centre;
}


void MomentFinder::setNumTempLines(int newNum)
{
    numTempLines = newNum;
}

