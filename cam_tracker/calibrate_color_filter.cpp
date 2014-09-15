#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "util.h"

using namespace cv;

static const char *USAGE = "\
Usage: calibrate_color_filter using URI\n\
\n\
URI must be an MJPEG stream.\n";

Point2i calculate_car_center(Mat mask)
{
    static Point2i points[640 * 480];

    // Calculate the mean point.
    int x_sum = 0, y_sum = 0, num_points = 0;
    for (int y = 0; y < mask.rows; ++y) {
        for (int x = 0; x < mask.cols; ++x) {
            if (mask.at<uchar>(y, x)) {
                points[num_points].x = x;
                points[num_points].y = y;
                x_sum += x;
                y_sum += y;
                ++num_points;
            }
        }
    }
    if (num_points == 0) {
        // Exit early if no points were found.
        return Point2i();
    }
    int x_mean = x_sum / num_points;
    int y_mean = y_sum / num_points;
    printf("%d, %d\n", x_mean, y_mean);

    // Calculate the variance.
    double variance = 0;
    for (int i = 0; i < num_points; ++i) {
        int x_distance = points[i].x - x_mean;
        int y_distance = points[i].y - y_mean;
        double squared_distance = x_distance * x_distance
                + y_distance * y_distance;
        variance += squared_distance / num_points;
    }

    // Now calculate the mean of the points within one variance of the
    // first mean.
    x_sum = 0, y_sum = 0;
    int num_filtered_points = 0;
    for (int i = 0; i < num_points; ++i) {
        int x_distance = points[i].x - x_mean;
        int y_distance = points[i].y - y_mean;
        int squared_distance = x_distance * x_distance
                + y_distance * y_distance;
        if (squared_distance <= variance) {
            x_sum += points[i].x;
            y_sum += points[i].y;
            ++num_filtered_points;
        }
    }
    x_mean = x_sum / num_filtered_points;
    y_mean = y_sum / num_filtered_points;

    printf("%d, %d\n", x_mean, y_mean);

    return Point2i(x_mean, y_mean);
}

int main(int argc, char **argv)
{
    if (argc != 3 || strcmp(argv[1], "using")) {
        fprintf(stderr, "%s", USAGE);
        exit(1);
    }

    const char *MASK_WINDOW = "Mask";
    const char *OUTPUT_WINDOW = "Output";
    VideoCapture capture = open_stream(argv[2]);
    namedWindow(MASK_WINDOW, WINDOW_AUTOSIZE);
    namedWindow(OUTPUT_WINDOW, WINDOW_AUTOSIZE);

    int h_min = 7, h_max = 15;
    int s_min = 172, s_max = 255;
    int v_min = 210, v_max = 255;
    createTrackbar("H min", MASK_WINDOW, &h_min, 127);
    createTrackbar("H max", MASK_WINDOW, &h_max, 127);
    createTrackbar("S min", MASK_WINDOW, &s_min, 255);
    //createTrackbar("S max", MASK_WINDOW, &s_max, 255);
    createTrackbar("V min", MASK_WINDOW, &v_min, 255);
    //createTrackbar("V max", MASK_WINDOW, &v_max, 255);

    Mat bgr_frame;
    int key = -1;
    const Scalar BLACK(0, 0, 0);
    while (key == -1 && capture.read(bgr_frame)) {
        Mat hsv_frame;
        cvtColor(bgr_frame, hsv_frame, CV_BGR2HSV);
        Mat mask, filtered;
        inRange(hsv_frame,
                Scalar(h_min, s_min, v_min),
                Scalar(h_max, s_max, v_max),
                mask);
        Mat eroded_mask;
        erode(mask, eroded_mask, Mat());
        Mat dilated_mask;
        dilate(eroded_mask, dilated_mask, Mat());
        bitwise_and(bgr_frame, bgr_frame, filtered, dilated_mask);
        Point2i center = calculate_car_center(dilated_mask);
        if (center.x != 0 && center.y != 0) {
            //fprintf(stderr, "%d, %d\n", center.x, center.y);
            circle(bgr_frame, center, 5, BLACK, -1);
        }
        imshow(MASK_WINDOW, filtered);
        imshow(OUTPUT_WINDOW, bgr_frame);
        key = waitKey(1);
    }
}

