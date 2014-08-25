#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
 
using namespace cv;
using namespace std;
 
 
RNG rng(12345);
 
void find_moments(Mat src);
 
int main(int argc, char **argv)
{
    // Load source image, convert it to gray and blur it.
    Mat src;
    src = imread("shapes.jpg", 1);
    namedWindow("Source", CV_WINDOW_AUTOSIZE);
    imshow("Source", src);

    Mat gray;
    cvtColor(src, gray, CV_BGR2GRAY);
    namedWindow("Gray", CV_WINDOW_AUTOSIZE);
    imshow("Gray", gray);
 
//    Mat blurred;
//    blur(gray, blurred, Size(3, 3));
//    namedWindow("Blurred", CV_WINDOW_AUTOSIZE);
//    imshow("Blurred", blurred);

    /*
    Mat eroded;
    erode(gray, eroded, Mat());
    namedWindow("Eroded", CV_WINDOW_AUTOSIZE);
    imshow("Eroded", eroded);

    Mat dilated;
    dilate(eroded, dilated, Mat());
    namedWindow("Dilated", CV_WINDOW_AUTOSIZE);
    imshow("Dilated", dilated);
    */
 
    //find_moments(dilated);
    find_moments(gray);
 
    waitKey(0);

    return 0;
}
 
void find_moments(Mat gray)
{
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    /*
    // Detect edges using canny
    Mat canny_output;
    Canny(gray, canny_output, 50, 150, 3);
    namedWindow("Canny", CV_WINDOW_AUTOSIZE);
    imshow("Canny", canny_output);
    */

    // Find contours
    //findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    //findContours(gray, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    findContours(gray, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0, 0));
 
    // Get the moments
    vector<Moments> mu(contours.size());
    for (int i = 0; i < contours.size(); ++i) {
        mu[i] = moments(contours[i], false);
    }
 
    // Get the mass centers
    vector<Point2f> mc(contours.size());
    for (int i = 0; i < contours.size(); ++i) {
        mc[i] = Point2f(mu[i].m10 / mu[i].m00 , mu[i].m01 / mu[i].m00);
    }
 
    // Draw contours
    //Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
    Mat drawing = Mat::zeros(gray.size(), CV_8UC3);
    for (int i = 0; i < contours.size(); ++i) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
        circle(drawing, mc[i], 4, color, -1, 8, 0);
    }
 
    // Show in a window
    namedWindow("Contours", CV_WINDOW_AUTOSIZE);
    imshow("Contours", drawing);
 
    // Calculate the area with the moments 00 and compare with the result of the OpenCV function
    printf("\t Info: Area and Contour Length \n");
    for (int i = 0; i < contours.size(); ++i) {
        printf(" * Contour[%d] - Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n", i, mu[i].m00, contourArea(contours[i]), arcLength(contours[i], true));
    }
}

