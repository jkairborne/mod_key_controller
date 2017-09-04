/* This is the moment finder class. It takes in an image and calculates out the image moments.
 * It then draws the center of mass onto another image which can then be output.
 * These lines being drawn can either be temporary (limited to a certain number) or permanent.
*/

#ifndef MOMENTFINDER_H
#define MOMENTFINDER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


class MomentFinder
{
    cv::Mat img, linesImg;
    int numTempLines, iLastX, iLastY, posX, posY;
    double dM01, dM10, dArea;
    bool linesOn, linesTemp, firstImg;
    std::vector<cv::Point> lastPts;

    //Function declarations:
    void drawLine();
    void calculateMoments();
public:
    MomentFinder(bool modifyimage = true, bool tempmodify = false, int nLines = 20);
    //Function declarations:
    // Function to be called to update an image. This function will then call the calculateMoments and drawLine functions.
    cv::Mat updateImg(cv::Mat& updImg);
    cv::Point getCentre();
    //Function to modify the number of temporary lines to be kept.
    void setNumTempLines(int newNum);
};

#endif // MOMENTFINDER_H
